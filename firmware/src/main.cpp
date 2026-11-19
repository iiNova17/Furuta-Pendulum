#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include "engine.h"
#include "hardware.h"
#include "onboard.h"

using namespace fw;
Controller control;
// Onboard closed-loop controller (product mode): runs every 4th 1 ms tick of an onboard run,
// from the encoder count and the latest AS5600 reading; its duty request then goes through the
// same safety path (Controller::tick) as an experiment.
OnboardController onboard;OnboardParams onboardParams;unsigned onboardTick=0,onboardEvery=4;
// 1.7: the AS5600 is read every kPendPeriodMs and its slow filter is set at boot (CONF bits SF,
// 0 = 16x ... 3 = 2x; datasheet settling 2.2 / 1.1 / 0.55 / 0.29 ms). Fast filter (FTH) stays off.
constexpr int kPendPeriodMs=1;
constexpr uint8_t kAs5600SlowFilter=0;   // 16x (the chip default): closed-loop simulation with datasheet noise showed
                                        // no delay-margin gain from 8x/4x/2x, only more noise (2026-11-19)
int as5600Conf=-1;          // CONF register read back after setting it (-1 = not written/read)
uint32_t pendReads=0,pendFails=0,pendReadMaxUs=0;   // AS5600 reads done / failed, longest I2C read [us]
SemaphoreHandle_t lock;
QueueHandle_t streamQueue,eventQueue;
TaskHandle_t adcTaskHandle,controlTaskHandle,pendTaskHandle;
esp_timer_handle_t adcTimer,controlTimer,pendTimer;
// The pendulum read is started kPendLeadUs before each control tick (its own esp_timer, same period), so it
// has finished when the controller uses it. Measured on the rig (12 V off, 250 Hz stream): age of the reading at
// the record, median / 99 %: FreeRTOS tick 0.94 / 1.68 ms, lead 350 us 0.88 / 0.90, 800 us 0.56 / 1.29,
// 950 us 0.45 / 1.45 ms. Reads take up to ~1 ms (I2C 400 kHz, preempted by the ADC task on core 0).
constexpr uint32_t kPendLeadUs=950;
hw::Snapshot sample;
Filter currentFilter;
uint64_t bootId,activeSession=0;
uint64_t challenge=0,lastHelloSession=0;
uint32_t lastHelloId=0,lastOpenId=0,highestCommandId=0;
uint32_t configId=1,packetSequence=0,dropped=0,streamHz=250,adcOverruns=0,controlOverruns=0;
// ADC oversampling (firmware 1.2): adcOversample back-to-back conversions are
// averaged per ADC tick. The count is chosen from the measured per-read time
// so the reads occupy at most kOversampleBudget of the ADC period.
constexpr double kOversampleBudget=.4;constexpr unsigned kOversampleMax=64;
volatile unsigned adcOversample=1;float adcReadUs=0;uint32_t adcMaxBusyUs=0,adcMaxGapUs=0,adcMaxLockWaitUs=0,adcMaxWakeUs=0;
volatile uint64_t adcFiredAt=0;
bool streaming=false,experimentValid=false,calibrating=false;
uint64_t calibrationStart=0;uint32_t calN=0,calClips=0;double calMean=0,calM2=0;
struct Notice { char text[96]; };
struct BurstBuffer {
  hw::AdcRecord* adc=nullptr;hw::Snapshot* core=nullptr;
  size_t adcCapacity=0,coreCapacity=0,adcCount=0,coreCount=0;
  uint64_t start=0,until=0;bool active=false,pending=false,overflow=false;double duration=0;
} burst;
struct Cached {uint32_t id=0;std::string request,response;std::vector<uint8_t> binary;};
Cached cache[16];unsigned cacheIndex=0;
std::vector<Segment> staged;
std::vector<double> tableUpload;

void notice(const char*s) {Notice n{};strncpy(n.text,s,sizeof(n.text)-1);xQueueSend(eventQueue,&n,0);}
bool armed(){return control.state==State::Armed||control.state==State::Running;}
void halt(){control.stop();hw::output(0);experimentValid=false;burst.pending=false;}
unsigned chooseOversample(uint32_t adcHz) {
  double budget=kOversampleBudget*1e6/adcHz;unsigned n=1;
  while(n*2<=kOversampleMax&&adcReadUs>0&&(n*2)*adcReadUs<=budget)n*=2;
  return n;
}
void measureAdcRead() {
  // Average over 256 conversions; called once at boot before tasks start.
  for(int i=0;i<16;i++)adc1_get_raw(ADC1_CHANNEL_3);
  uint64_t t0=esp_timer_get_time();for(int i=0;i<256;i++)adc1_get_raw(ADC1_CHANNEL_3);
  adcReadUs=(esp_timer_get_time()-t0)/256.f;
}
void applyConfig(const Config& c) {
  control.cfg=c;currentFilter.reset(c);configId++;streamHz=c.streamHz;adcOversample=chooseOversample(c.adcHz);adcMaxBusyUs=adcMaxGapUs=adcMaxLockWaitUs=adcMaxWakeUs=0;
  esp_timer_stop(adcTimer);esp_timer_start_periodic(adcTimer,1000000/c.adcHz);
  ledcSetup(0,c.pwmHz,10);sample.filtered=NAN;
}
void configJson(JsonObject o,const Config&c) {
  o["schema"]=1;o["maxDuty"]=c.maxDuty;o["slewPerSecond"]=c.slewPerSecond;o["maxDuration"]=c.maxDuration;
  o["travelCounts"]=c.travelCounts;o["velocityCountsPerSecond"]=c.velocityCountsPerSecond;
  o["currentTripA"]=c.currentTripA;o["currentTripMs"]=c.currentTripMs;
  o["currentZeroMv"]=c.currentZeroMv;o["sensitivityMvA"]=c.sensitivityMvA;o["countsPerRev"]=c.countsPerRev;
  o["armSign"]=c.armSign;o["pendSign"]=c.pendSign;
  if(c.pendDownRaw>=0)o["pendDownRaw"]=c.pendDownRaw;else o["pendDownRaw"]=nullptr;
  o["deadzonePos"]=c.deadzonePos;o["deadzoneNeg"]=c.deadzoneNeg;
  o["assumedBusV"]=c.assumedBusV;o["heartbeatMs"]=c.heartbeatMs;o["streamHz"]=c.streamHz;
  o["adcHz"]=c.adcHz;o["pwmHz"]=c.pwmHz;
  auto f=o.createNestedObject("filter");f["type"]=c.filterType;f["alpha"]=c.alpha;f["window"]=c.window;
  auto a=f.createNestedArray("sos");for(double x:c.sos)a.add(x);
}
bool parseConfig(JsonObjectConst o,Config&c,std::string&w) {
  if(o.isNull()){w="Config object required";return false;}
  const char* keys[]={"schema","maxDuty","slewPerSecond","maxDuration","travelCounts","velocityCountsPerSecond",
    "currentTripA","currentTripMs","currentZeroMv","sensitivityMvA","countsPerRev","armSign","pendSign","pendDownRaw",
    "assumedBusV","heartbeatMs","streamHz","adcHz","pwmHz","filter","deadzonePos","deadzoneNeg"};
  for(auto kv:o){bool known=false;for(auto k:keys)if(!strcmp(k,kv.key().c_str()))known=true;if(!known){w="Unknown config field";return false;}}
  auto number=[&](const char*k,double&v,bool nullable=false){
    if(!o.containsKey(k))return true;
    if(o[k].isNull()&&nullable){v=NAN;return true;}
    if(!o[k].is<double>())return false;v=o[k].as<double>();return std::isfinite(v);
  };
  if(o.containsKey("schema")&&o["schema"]!=1){w="Unsupported schema";return false;}
  if(!number("maxDuty",c.maxDuty)||!number("slewPerSecond",c.slewPerSecond)||!number("maxDuration",c.maxDuration)||
     !number("travelCounts",c.travelCounts)||!number("velocityCountsPerSecond",c.velocityCountsPerSecond)||
     !number("currentTripA",c.currentTripA)||!number("currentTripMs",c.currentTripMs)||!number("currentZeroMv",c.currentZeroMv,true)||
     !number("sensitivityMvA",c.sensitivityMvA)||!number("countsPerRev",c.countsPerRev,true)||!number("deadzonePos",c.deadzonePos,true)||!number("deadzoneNeg",c.deadzoneNeg,true)||!number("assumedBusV",c.assumedBusV)) {w="Expected finite numeric config";return false;}
  auto integer=[&](const char*k,auto&v){
    if(!o.containsKey(k))return true;
    if(!o[k].is<double>())return false;double x=o[k].as<double>();
    if(!std::isfinite(x)||std::floor(x)!=x||x< -1||x>1000000)return false;v=x;return true;
  };
  if(!integer("armSign",c.armSign)||!integer("pendSign",c.pendSign)||!integer("heartbeatMs",c.heartbeatMs)||
     !integer("streamHz",c.streamHz)||!integer("adcHz",c.adcHz)||!integer("pwmHz",c.pwmHz)){w="Expected integer config";return false;}
  if(o.containsKey("pendDownRaw")){if(o["pendDownRaw"].isNull())c.pendDownRaw=-1;else if(!integer("pendDownRaw",c.pendDownRaw)){w="Invalid pendulum zero";return false;}}
  if(o.containsKey("filter")) {
    auto f=o["filter"].as<JsonObjectConst>();if(f.isNull()){w="Expected filter object";return false;}
    c.filterType=f["type"]|c.filterType.c_str();
    if(f.containsKey("alpha"))c.alpha=f["alpha"].as<double>();
    if(f.containsKey("window"))c.window=f["window"].as<unsigned>();
    if(f.containsKey("sos")){auto a=f["sos"].as<JsonArrayConst>();if(a.size()!=6){w="Six SOS coefficients required";return false;}for(int i=0;i<6;i++)c.sos[i]=a[i].as<double>();}
  }
  return c.valid(w);
}
// Onboard controller knobs by name (JSON <-> OnboardParams). Defaults come from onboard_design.h.
struct NamedValue {const char* name;double* value;};
std::vector<NamedValue> onboardFields(OnboardParams&p) {
  return {{"K0",&p.K[0]},{"K1",&p.K[1]},{"K2",&p.K[2]},{"K3",&p.K[3]},{"velFc",&p.velFc},{"r",&p.r},
    {"fricGain",&p.fricGain},{"fricBand",&p.fricBand},{"catchTh1",&p.catchTh1},{"catchW1",&p.catchW1},
    {"exitTh1",&p.exitTh1},{"safeTh0",&p.safeTh0},{"E0",&p.E0},{"kE",&p.kE},{"aPush",&p.aPush},{"wc",&p.wc},
    {"zc",&p.zc},{"aKick",&p.aKick},{"kickE",&p.kickE},{"wSign",&p.wSign},{"wFric",&p.wFric},{"iMax",&p.iMax},
    {"uMax",&p.uMax},{"tsp",&p.tsp},{"tsn",&p.tsn},{"stickGain",&p.stickGain},{"wStick",&p.wStick},{"aStick",&p.aStick}};
}
void onboardJson(JsonObject o,OnboardParams p) {
  for(auto&f:onboardFields(p))o[f.name]=*f.value;
  o["swingUp"]=p.swingUp;o["rearm"]=p.rearm;o["Ts"]=p.Ts;
  o["modelArmSign"]=design::modelArmSign;o["modelPendSign"]=design::modelPendSign;
}
bool parseOnboard(JsonObjectConst o,OnboardParams&p,std::string&w) {
  if(o.isNull()){w="Params object required";return false;}
  auto fields=onboardFields(p);
  // Ts first: it loads the design's LQR gain for that period; explicit K0..K3 in the same message win.
  if(o.containsKey("Ts")){
    if(!o["Ts"].is<double>()||!p.setPeriod(o["Ts"].as<double>())){w="Ts must be 0.001, 0.002 or 0.004";return false;}
  }
  for(auto kv:o){
    const char* k=kv.key().c_str();
    if(!strcmp(k,"Ts"))continue;
    if(!strcmp(k,"swingUp")){if(!kv.value().is<bool>()){w="swingUp must be true/false";return false;}p.swingUp=kv.value().as<bool>();continue;}
    bool known=false;
    for(auto&f:fields)if(!strcmp(f.name,k)){
      if(!kv.value().is<double>()){w="Expected numeric onboard parameter";return false;}
      *f.value=kv.value().as<double>();known=true;
    }
    if(!known){w="Unknown onboard parameter";return false;}
  }
  return p.valid(w);
}
bool parseSegment(JsonObjectConst o,Segment&s,std::string&w) {
  if(o.isNull()){w="Segment object required";return false;}
  s.kind=o["kind"]|"hold";
  auto n=[&](const char*k,double&x){if(!o.containsKey(k))return true;if(!o[k].is<double>())return false;x=o[k].as<double>();return std::isfinite(x);};
  if(!n("duration",s.duration)||!n("amplitude",s.amplitude)||!n("offset",s.offset)||!n("f0",s.f0)||!n("f1",s.f1)||
     !n("phase",s.phase)||!n("duty",s.duty)||!n("end",s.end)||!n("bitPeriod",s.bitPeriod)||!n("tablePeriod",s.tablePeriod)){w="Invalid waveform numeric field";return false;}
  s.seed=o["seed"]|1;s.logarithmic=o["logarithmic"]|false;s.linear=o["linear"]|false;
  auto vec=[&](const char*k,std::vector<double>&v){if(!o.containsKey(k))return true;auto a=o[k].as<JsonArrayConst>();if(a.isNull()||a.size()>4096)return false;for(auto x:a){if(!x.is<double>()||!std::isfinite(x.as<double>()))return false;v.push_back(x.as<double>());}return true;};
  if(!vec("frequencies",s.frequencies)||!vec("amplitudes",s.amplitudes)||!vec("phases",s.phases)||!vec("table",s.table)){w="Invalid waveform vector";return false;}
  if(s.kind=="arbitrary"&&s.table.empty())s.table=tableUpload;
  return s.validate(control.cfg,w);
}
// Status is copied under the mutex (fast, no allocation) and serialized to
// JSON afterwards, so host commands never hold the lock the ADC task needs.
struct StatusCopy {
  State state;std::string fault;bool centered,encoderOk,pendValid,burstActive,burstPending,burstOverflow;
  uint32_t configId,runId,dropped,adcOverruns,controlOverruns,adcOversample,adcMaxBusyUs,adcMaxGapUs,adcMaxLockWaitUs,adcMaxWakeUs;
  uint64_t deviceUs;int64_t center;uint8_t pendStatus;uint32_t pendReads,pendFails,pendReadMaxUs;float currentMv,filteredMv,adcReadUs;size_t burstAdcCount,burstCoreCount;int calSource;
};
StatusCopy statusCopy() {
  StatusCopy c;c.state=control.state;c.fault=control.fault;c.centered=control.centered;c.encoderOk=hw::encoderOk;
  c.pendValid=sample.flags&2;c.burstActive=burst.active;c.burstPending=burst.pending;c.burstOverflow=burst.overflow;
  c.configId=configId;c.runId=control.run;c.dropped=dropped;c.adcOverruns=adcOverruns;c.controlOverruns=controlOverruns;
  c.adcOversample=adcOversample;c.adcMaxBusyUs=adcMaxBusyUs;c.adcMaxGapUs=adcMaxGapUs;c.adcMaxLockWaitUs=adcMaxLockWaitUs;c.adcMaxWakeUs=adcMaxWakeUs;
  c.deviceUs=esp_timer_get_time();c.center=control.center;c.pendReads=pendReads;c.pendFails=pendFails;c.pendReadMaxUs=pendReadMaxUs;c.pendStatus=sample.status;c.currentMv=sample.mv;c.filteredMv=sample.filtered;
  c.adcReadUs=adcReadUs;c.burstAdcCount=burst.adcCount;c.burstCoreCount=burst.coreCount;c.calSource=(int)hw::calibrationSource;
  return c;
}
void statusJson(JsonObject o,const StatusCopy&c) {
  o["state"]=stateName(c.state);o["fault"]=c.fault;o["centered"]=c.centered;
  o["configId"]=c.configId;o["runId"]=c.runId;o["bootId"]=std::to_string(bootId);
  o["deviceUs"]=std::to_string(c.deviceUs);o["dropped"]=c.dropped;
  o["adcOverruns"]=c.adcOverruns;o["controlOverruns"]=c.controlOverruns;
  o["encoderOk"]=c.encoderOk;o["pendStatus"]=c.pendStatus;o["pendValid"]=c.pendValid;
  o["pendReads"]=c.pendReads;o["pendFails"]=c.pendFails;o["pendReadMaxUs"]=c.pendReadMaxUs;
  o["currentMv"]=c.currentMv;o["filteredMv"]=c.filteredMv;o["centerCount"]=std::to_string(c.center);
  o["burstActive"]=c.burstActive;o["burstPending"]=c.burstPending;
  o["burstAdcCount"]=c.burstAdcCount;o["burstCoreCount"]=c.burstCoreCount;o["burstOverflow"]=c.burstOverflow;
  o["adcCalibrationSource"]=c.calSource;
  o["adcOversample"]=c.adcOversample;o["adcReadUs"]=c.adcReadUs;o["adcMaxBusyUs"]=c.adcMaxBusyUs;o["adcMaxGapUs"]=c.adcMaxGapUs;
  o["adcMaxLockWaitUs"]=c.adcMaxLockWaitUs;o["adcMaxWakeUs"]=c.adcMaxWakeUs;
}
bool sendPacket(uint8_t type,const std::vector<uint8_t>& data,bool critical=false) {
  Packet p;p.type=type;p.session=activeSession;p.sequence=packetSequence++;p.payload=data;
  auto bytes=encode(p);if(bytes.empty())return false;
  if(!critical&&Serial.availableForWrite()<(int)bytes.size()){
    xSemaphoreTake(lock,portMAX_DELAY);
    dropped+=(type==Samples&&data.size()>=22)?get<uint16_t>(&data[20]):1;
    xSemaphoreGive(lock);return false;
  }
  Serial.write(bytes.data(),bytes.size());return true;
}
void sendJson(uint8_t type,const std::string&s){std::vector<uint8_t>p(s.begin(),s.end());sendPacket(type,p,true);}
void startBurst(uint64_t now){burst.start=now;burst.until=now+(uint64_t)(burst.duration*1e6);burst.active=true;burst.pending=false;}
void adcTask(void*) {
  while(true) {
    uint32_t n=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    static uint64_t lastWake=0;
    uint64_t t=esp_timer_get_time();const unsigned k=adcOversample;
    {uint64_t f=adcFiredAt;if(f&&t>f&&t-f>adcMaxWakeUs)adcMaxWakeUs=t-f;}
    if(lastWake&&t-lastWake>adcMaxGapUs)adcMaxGapUs=t-lastWake;
    lastWake=t;
    uint32_t sum=0;int lo=4095,hi=0;
    for(unsigned i=0;i<k;i++){int r=adc1_get_raw(ADC1_CHANNEL_3);sum+=r;lo=std::min(lo,r);hi=std::max(hi,r);}
    // Timestamp the centre of the averaging window.
    uint64_t done=esp_timer_get_time();if(done-t>adcMaxBusyUs)adcMaxBusyUs=done-t;t=(t+done)/2;
    // Linear interpolation between neighbouring calibrated codes keeps the
    // sub-LSB resolution gained by averaging.
    unsigned base=sum/k;double frac=double(sum)/k-base;
    double mvLo=esp_adc_cal_raw_to_voltage(base,&hw::adcCalibration);
    double mvHi=base<4095?esp_adc_cal_raw_to_voltage(base+1,&hw::adcCalibration):mvLo;
    float mv=mvLo+(mvHi-mvLo)*frac;
    // Any clipped conversion inside the window is reported as clipped.
    int raw=lo<17?lo:(hi>4078?hi:(int)std::lround(double(sum)/k));
    uint64_t w0=esp_timer_get_time();xSemaphoreTake(lock,portMAX_DELAY);
    {uint64_t w=esp_timer_get_time()-w0;if(w>adcMaxLockWaitUs)adcMaxLockWaitUs=w;}
    if(n>1)adcOverruns+=n-1;
    sample.adc=raw;sample.mv=mv;sample.adcTime=t;sample.filtered=currentFilter.step(mv);
    // t is taken before the mutex, so a sample can predate a calibration
    // started meanwhile; skip it (an unsigned t-start would wrap and end the
    // 1 s window after one sample).
    if(calibrating&&t>=calibrationStart){
      calN++;double d=mv-calMean;calMean+=d/calN;calM2+=d*(mv-calMean);if(raw<17||raw>4078)calClips++;
      if(t-calibrationStart>=1000000){
        calibrating=false;
        if(calN>=100&&calMean>=2200&&calMean<=2800&&!calClips){
          control.cfg.currentZeroMv=calMean;configId++;notice("CURRENT_ZERO_UPDATED");
        }else notice("CURRENT_ZERO_REJECTED");
      }
    }
    if(burst.active){
      if(t>=burst.until){burst.active=false;notice("BURST_COMPLETE");}
      else if(burst.adcCount<burst.adcCapacity)burst.adc[burst.adcCount++]={t,(uint16_t)raw,mv,sample.requested,sample.applied};
      else{burst.active=false;burst.overflow=true;control.fail("BURST_OVERFLOW");hw::output(0);notice("BURST_OVERFLOW");}
    }
    xSemaphoreGive(lock);
  }
}
void controlTask(void*) {
  esp_task_wdt_add(nullptr);
  int64_t prev=hw::count();uint64_t prevAt=esp_timer_get_time();unsigned tick=0,overrunStreak=0;
  while(true){
    uint32_t n=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(n>1){overrunStreak++;}else overrunStreak=0;
    uint64_t t=esp_timer_get_time();int64_t count=hw::count();double cps=(count-prev)*1e6/std::max<uint64_t>(1,t-prevAt);prev=count;prevAt=t;
    xSemaphoreTake(lock,portMAX_DELAY);
    if(n>1)controlOverruns+=n-1;
    t=esp_timer_get_time(); // Ages must use time after the mutex was acquired.
    State before=control.state;
    if((n>5||overrunStreak>=5)&&armed())control.fail("CONTROL_OVERRUN");
    if(control.state==State::Running&&control.source==Controller::Source::Onboard) {
      if(onboardTick++%onboardEvery==0) {            // controller period Ts (1, 2 or 4 ms), held in between
        const bool pendFresh=t-sample.pendTime<20000&&(sample.status&0x20);
        if(!pendFresh){control.fail("PEND_INVALID");notice("PEND_INVALID");}
        else{
          const double th0=armAngle(count,control.center,control.cfg.armSign,control.cfg.countsPerRev);
          const double th1=pendAngle(sample.pend,control.cfg.pendDownRaw,control.cfg.pendSign);
          control.external=onboard.step(th0,th1);
          if(onboard.mode==OnboardController::SafeStop){control.stop();notice("ONBOARD_SAFE_STOP");}
          else if(!onboard.p.swingUp&&onboard.mode==OnboardController::SwingUp){control.stop();notice("ONBOARD_PENDULUM_LOST");}
        }
      }
    }
    // 1.5: no trip on a single clipped conversion (switching-noise spikes tripped real runs at 0.74 A).
    // The averaged-reading check in Controller::tick (ADC_INVALID_OR_CLIPPED) remains.
    control.tick(t,count,cps,sample.mv,t-sample.adcTime,hw::encoderOk,sample.filtered);
    const bool onboardRunning=control.state==State::Running&&control.source==Controller::Source::Onboard;
    sample.ctrl=onboardRunning?(uint8_t)(onboard.mode+1):0;
    if(burst.pending&&control.state==State::Running)startBurst(t);
    hw::output(control.applied);
    esp_task_wdt_reset();
    t=esp_timer_get_time();
    sample.applyTime=t;sample.count=count;sample.requested=control.requested;sample.applied=control.applied;sample.state=(uint8_t)control.state;
    sample.limits=control.flags;
    if(before!=control.state)notice(stateName(control.state));
    if(++tick%2==0){
      sample.t=t;sample.seq++;sample.configId=configId;sample.runId=control.run;sample.flags=(hw::encoderOk?1:0)|((t-sample.pendTime<20000&&(sample.status&0x20))?2:0)|
        ((t-sample.adcTime<20000&&sample.adc>16&&sample.adc<4079)?4:0)|((sample.status&0x18)?8:0);
      if(burst.active) {
        if(burst.coreCount<burst.coreCapacity)burst.core[burst.coreCount++]=sample;
        else{burst.active=false;burst.overflow=true;control.fail("BURST_CORE_OVERFLOW");hw::output(0);}
      }
      unsigned stride=500/streamHz;
      if(streaming&&sample.seq%stride==0)if(xQueueSend(streamQueue,&sample,0)!=pdTRUE)dropped++;
    }
    xSemaphoreGive(lock);
  }
}
void pendulumTask(void*) {
  unsigned cycle=0;uint8_t status=0,agc=0;uint16_t mag=0;
  while(true){
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);     // pendTimer: every kPendPeriodMs, kPendLeadUs before the control tick
    const uint64_t r0=esp_timer_get_time();
    uint8_t a[2],b[1],d[3];bool good=hw::readReg(0x0c,a,2);
    const uint32_t readUs=(uint32_t)(esp_timer_get_time()-r0);
    if(++cycle%25==0){if(hw::readReg(0x0b,b,1))status=b[0];else good=false;if(hw::readReg(0x1a,d,3)){agc=d[0];mag=((d[1]&15)<<8)|d[2];}}
    uint64_t t=esp_timer_get_time();
    xSemaphoreTake(lock,portMAX_DELAY);
    pendReads++;if(!good)pendFails++;if(readUs>pendReadMaxUs)pendReadMaxUs=readUs;
    if(good){sample.pend=((a[0]&15)<<8)|a[1];sample.pendTime=t;sample.status=status;sample.agc=agc;sample.mag=mag;}
    else sample.status=0;
    xSemaphoreGive(lock);
  }
}
// Set the AS5600 slow filter (volatile CONF register, so on every boot) and read it back.
void configureAs5600() {
  uint8_t c[2];
  if(!hw::readReg(0x07,c,2))return;
  uint16_t conf=((uint16_t)c[0]<<8)|c[1];
  conf=(conf&~(uint16_t)((3u<<8)|(7u<<10)))|(uint16_t)((kAs5600SlowFilter&3u)<<8);   // SF = bits 9:8, FTH = bits 12:10 off
  Wire.beginTransmission(0x36);Wire.write(0x07);Wire.write((uint8_t)(conf>>8));Wire.write((uint8_t)(conf&0xff));
  if(Wire.endTransmission())return;
  delay(3);
  if(hw::readReg(0x07,c,2))as5600Conf=((int)c[0]<<8)|c[1];
}
void timerNotify(void*arg){xTaskNotifyGive(*(TaskHandle_t*)arg);}
void adcTimerNotify(void*arg){adcFiredAt=esp_timer_get_time();xTaskNotifyGive(*(TaskHandle_t*)arg);}

void dispatch(const Packet&p) {
  DynamicJsonDocument in(16384),out(8192);
  auto error=deserializeJson(in,p.payload.data(),p.payload.size(),DeserializationOption::NestingLimit(8));
  if(error||!in.is<JsonObject>())return;
  uint32_t id=in["id"]|uint32_t(0);std::string op=in["op"]|"";if(!id||op.empty())return;
  std::string request((const char*)p.payload.data(),p.payload.size());
  bool emergency=op=="STOP"||op=="DISARM"||op=="ABORT";
  auto response=out.to<JsonObject>();response["id"]=id;response["op"]=op;response["ok"]=true;
  std::string why;Config cfgOut;bool wantCfg=false;
  xSemaphoreTake(lock,portMAX_DELAY);
  if(emergency)halt();
  else if(op!="HELLO"&&op!="SESSION_OPEN"&&(!activeSession||p.session!=activeSession))why="Session mismatch";
  if(why.empty()&&!emergency&&op!="HELLO"&&op!="SESSION_OPEN"){
    for(const auto&c:cache)if(c.id==id){
      xSemaphoreGive(lock);
      if(c.request==request){if(!c.binary.empty())sendPacket(Burst,c.binary,true);else sendJson(Ack,c.response);}
      else {response["ok"]=false;response["error"]="Command ID reused with different payload";std::string s;serializeJson(out,s);sendJson(Ack,s);}
      return;
    }
    if(id<=highestCommandId)why="Stale command outside retry cache";
    else highestCommandId=id;
  }
  if(!why.empty()){}
  else if(emergency){}
  else if(op=="HELLO"){
    if(id!=lastHelloId||p.session!=lastHelloSession){challenge=((uint64_t)esp_random()<<32)|esp_random();lastHelloId=id;lastHelloSession=p.session;}
    response["challenge"]=std::to_string(challenge);
    response["firmware"]="FurutaWorkbench/1.8";response["runLengthUnlimited"]=true;response["onboardControl"]=true;response["protocol"]=1;response["coreRecordBytes"]=88;
    {JsonArray a=response.createNestedArray("onboardPeriods");for(int i=0;i<rates::count;i++)a.add(rates::Ts[i]);}
    response["pendHz"]=1000/kPendPeriodMs;response["as5600Conf"]=as5600Conf;response["as5600SlowFilter"]=kAs5600SlowFilter;
    response["adcChunkMax"]=64;response["coreChunkMax"]=16;
    response["baud"]=460800;response["flashBytes"]=ESP.getFlashChipSize();response["psramBytes"]=ESP.getPsramSize();
    response["maxBurstBytes"]=4194304;response["maxSegments"]=kMaxSegments;response["maxTable"]=4096;
    response["adcOversample"]=adcOversample;response["adcReadUs"]=adcReadUs;response["currentTripSource"]="filtered";
  }else if(op=="SESSION_OPEN"){
    std::string suppliedBoot=in["bootId"]|"",suppliedChallenge=in["challenge"]|"";
    if(!p.session||!challenge||suppliedBoot!=std::to_string(bootId)||suppliedChallenge!=std::to_string(challenge))why="Fresh HELLO challenge/boot ID required";
    else if(p.session==activeSession&&id==lastOpenId){}
    else{halt();activeSession=p.session;lastOpenId=id;highestCommandId=id;for(auto&c:cache)c={};streaming=false;}
  }else if(op=="HEARTBEAT"){
    if(armed())control.lastHeartbeat=esp_timer_get_time();
  }else if(op=="STATUS"||op=="CAPABILITIES"||op=="EXPERIMENT_STATUS"){
    if(in["resetDiagnostics"]|false){adcMaxBusyUs=adcMaxGapUs=adcMaxLockWaitUs=adcMaxWakeUs=0;}
  }
  else if(op=="GET_CONFIG"){cfgOut=control.cfg;wantCfg=true;}
  else if(op=="START_STREAM"){streaming=true;xQueueReset(streamQueue);}
  else if(op=="STOP_STREAM"){streaming=false;xQueueReset(streamQueue);}
  else if(op=="ARM"){
    if(calibrating||burst.active)why="Capture/calibration in progress";
    else if(!hw::encoderOk||sample.adcTime==0)why="Acquisition not ready";
    else control.arm(activeSession,esp_timer_get_time(),why);
  }else if(op=="START"){
    if(!experimentValid)why="Upload/validate an experiment first";
    else control.start(esp_timer_get_time(),why);
  }else if(op=="START_ONBOARD"){
    // Product mode: the ESP32 runs the hybrid controller. Gates: armed (centred arm, commissioned
    // limits, heartbeat), encoder scale, pendulum-down reference, fresh pendulum reading, and the
    // profile signs the model was identified with (a flipped sign would push the wrong way).
    OnboardParams p=onboardParams;
    if(in.containsKey("swingUp")){if(!in["swingUp"].is<bool>())why="swingUp must be true/false";else p.swingUp=in["swingUp"].as<bool>();}
    const Config&c=control.cfg;
    if(!why.empty()){}
    else if(control.state!=State::Armed)why="ARM required";
    else if(!std::isfinite(c.countsPerRev)||c.countsPerRev<=0)why="Encoder scale (countsPerRev) required";
    else if(c.pendDownRaw<0)why="Pendulum-down reference required";
    else if(!(esp_timer_get_time()-sample.pendTime<20000&&(sample.status&0x20)))why="Pendulum reading invalid";
    else if(c.armSign!=design::modelArmSign||c.pendSign!=design::modelPendSign)
      why="Profile armSign/pendSign differ from the identified model's ("+std::to_string(design::modelArmSign)+", "+std::to_string(design::modelPendSign)+")";
    else if(p.valid(why)){
      onboard.p=p;onboard.reset();onboardTick=0;onboardEvery=(unsigned)std::lround(p.Ts*1000);
      if(control.startExternal(esp_timer_get_time(),Controller::Source::Onboard,why)){
        response["mode"]=p.swingUp?"swing-up + LQR":"LQR only";response["seconds"]=c.maxDuration;response["Ts"]=p.Ts;
        if(c.slewPerSecond<50)response["warning"]="slewPerSecond is below 50/s; the controller was designed with 100/s";
      }
    }
  }else if(op=="ONBOARD_GET"){
    onboardJson(response.createNestedObject("params"),onboardParams);
  }else if(op=="ONBOARD_BENCH"){
    // CPU time of one controller step with the current knobs, on a private copy (no effect on runs).
    // Runs in the command loop, below the 1 ms control task, so preemption makes it a slight overestimate.
    // The command handler holds the shared lock: release it while timing, so the control, ADC and
    // pendulum tasks keep running (1.7 first build blocked them for the whole benchmark). Disarmed only.
    if(armed())why="Disarm before timing the controller";
    else{
      OnboardController b;b.p=onboardParams;b.reset();
      const int n=std::min(5000,std::max(100,(int)(in["steps"]|2000)));double acc=0,worst=0;
      xSemaphoreGive(lock);
      const uint64_t t0=esp_timer_get_time();
      for(int i=0;i<n;i++){const uint64_t s=esp_timer_get_time();acc+=b.step(.3*std::sin(i*.003),3.*std::sin(i*.01));worst=std::max(worst,(double)(esp_timer_get_time()-s));}
      const uint64_t t1=esp_timer_get_time();
      xSemaphoreTake(lock,portMAX_DELAY);
      response["steps"]=n;response["usPerStep"]=(t1-t0)/(double)n;response["usWorst"]=worst;response["check"]=acc;
    }
  }else if(op=="CLEAR_FAULT"){
    if(control.state==State::Fault){control.fault.clear();control.state=State::Disarmed;halt();}
  }else if(armed())why="Disarm before changing configuration/captures";
  else if(op=="SET_CONFIG"){
    if(calibrating||burst.active||burst.pending)why="Capture/calibration in progress";
    else{Config c=control.cfg;if(parseConfig(in["config"].as<JsonObjectConst>(),c,why)){applyConfig(c);experimentValid=false;cfgOut=c;wantCfg=true;}}
  }else if(op=="ONBOARD_CONFIG"){
    OnboardParams p=onboardParams;
    if(parseOnboard(in["params"].as<JsonObjectConst>(),p,why)){onboardParams=p;onboardJson(response.createNestedObject("params"),p);}
  }else if(op=="ONBOARD_DEFAULTS"){
    onboardParams=OnboardParams();onboardJson(response.createNestedObject("params"),onboardParams);
  }else if(op=="SAVE_CONFIG"){
    DynamicJsonDocument doc(4096);configJson(doc.to<JsonObject>(),control.cfg);std::string json;serializeJson(doc,json);
    Preferences pref;pref.begin("furuta",false);size_t n=pref.putString("config",json.c_str());pref.putUInt("crc",crc32((const uint8_t*)json.data(),json.size()));pref.end();
    if(!n)why="NVS write failed";
  }else if(op=="CALIBRATE"){
    std::string target=in["target"]|"";
    if(burst.active||calibrating)why="Capture/calibration in progress";
    else if(target=="arm"){
      // Centering zeroes the encoder count itself, so raw count reads 0 here.
      hw::zeroCount();control.center=0;control.centered=true;configId++;experimentValid=false;
    }else if(target=="down"||target=="upright"){
      if(!(sample.flags&2))why="Pendulum reading invalid";
      else{int raw=sample.pend;if(target=="upright")raw=(raw+2048)%4096;control.cfg.pendDownRaw=raw;configId++;}
    }else if(target=="current"){
      control.cfg.currentZeroMv=NAN;configId++;calN=calClips=0;calMean=calM2=0;calibrationStart=esp_timer_get_time();calibrating=true;
    }else why="Unknown calibration target";
  }else if(op=="TABLE_CLEAR")tableUpload.clear();
  else if(op=="TABLE_CHUNK"){
    size_t offset=in["offset"]|0;auto a=in["values"].as<JsonArrayConst>();
    if(offset!=tableUpload.size()||a.isNull()||a.size()>128||offset+a.size()>4096)why="Invalid table chunk";
    else {std::vector<double>v;for(auto x:a){double y=x.as<double>();if(!x.is<double>()||!std::isfinite(y)){why="Invalid table value";break;}v.push_back(y);}if(why.empty())tableUpload.insert(tableUpload.end(),v.begin(),v.end());}
  }else if(op=="UPLOAD_BEGIN"){staged.clear();experimentValid=false;}
  else if(op=="SET_SEGMENT"){
    if((size_t)(in["index"]|99999)!=staged.size()||staged.size()>=kMaxSegments)why="Segments must be uploaded in order";
    else {Segment s;if(parseSegment(in["segment"].as<JsonObjectConst>(),s,why))staged.push_back(std::move(s));}
  }else if(op=="VALIDATE"||op=="UPLOAD_EXPERIMENT"){
    experimentValid=false;Experiment e;e.segments=staged;if(e.validate(control.cfg,why)){control.experiment=std::move(e);experimentValid=true;response["duration"]=control.experiment.duration();}
  }else if(op=="CAPTURE_BURST"){
    double seconds=in["seconds"]|1.;bool pending=in["withRun"]|false;
    size_t na=(size_t)std::ceil(seconds*control.cfg.adcHz)+8,nc=(size_t)std::ceil(seconds*500)+8;
    if(!std::isfinite(seconds)||seconds<=0||seconds>120||na*sizeof(hw::AdcRecord)+nc*sizeof(hw::Snapshot)>4194304)why="Capture exceeds finite memory budget";
    else if(burst.active||calibrating)why="Capture/calibration in progress";
    else{
      free(burst.adc);free(burst.core);burst={};
      burst.adc=(hw::AdcRecord*)heap_caps_malloc(na*sizeof(hw::AdcRecord),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
      burst.core=(hw::Snapshot*)heap_caps_malloc(nc*sizeof(hw::Snapshot),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
      if(!burst.adc||!burst.core){
        free(burst.adc);free(burst.core);
        burst.adc=(hw::AdcRecord*)malloc(na*sizeof(hw::AdcRecord));burst.core=(hw::Snapshot*)malloc(nc*sizeof(hw::Snapshot));
      }
      if(!burst.adc||!burst.core){free(burst.adc);free(burst.core);burst={};why="Capture allocation failed: reduce duration";}
      else{burst.adcCapacity=na;burst.coreCapacity=nc;burst.duration=seconds;burst.pending=pending;if(!pending)startBurst(esp_timer_get_time());}
    }
  }else if(op=="BURST_INFO"){
    response["adcCount"]=burst.adcCount;response["coreCount"]=burst.coreCount;response["active"]=burst.active;response["pending"]=burst.pending;
    response["overflow"]=burst.overflow;response["adcHz"]=control.cfg.adcHz;response["startUs"]=std::to_string(burst.start);
  }else if(op=="READ_CHUNK"){
    if(burst.active||burst.pending)why="Wait for capture completion";
    else{
      bool core=(in["channel"]|"adc")==std::string("core");
      size_t offset=in["offset"]|0,n=std::min((size_t)(in["count"]|16),(size_t)(core?16:64));
      size_t total=core?burst.coreCount:burst.adcCount;if(offset>total)why="Offset out of range";
      else{
        n=std::min(n,total-offset);
        // The finished burst buffer is only modified by CAPTURE_BURST, which
        // runs in this same task, so it is packed outside the mutex.
        xSemaphoreGive(lock);
        std::vector<uint8_t> data;data.reserve(11+n*sizeof(hw::Snapshot));
        put<uint32_t>(data,id);put<uint32_t>(data,offset);put<uint16_t>(data,n);put<uint8_t>(data,core?1:0);
        for(size_t i=0;i<n;i++){if(core)hw::pack(data,burst.core[offset+i]);else hw::pack(data,burst.adc[offset+i]);}
        cache[cacheIndex++%16]={id,request,"",data};sendPacket(Burst,data,true);return;
      }
    }
  }else why="Unknown operation";
  if(!why.empty()){response["ok"]=false;response["error"]=why;}
  StatusCopy sc=statusCopy();bool pendingCal=op=="CALIBRATE"&&calibrating;
  xSemaphoreGive(lock);
  if(wantCfg)configJson(response.createNestedObject("config"),cfgOut);
  statusJson(response.createNestedObject("status"),sc);
  if(pendingCal)response["pending"]=true;
  // JSON serialization and the retry cache stay outside the mutex so the
  // 2 kHz ADC task is not delayed by host commands.
  std::string reply;serializeJson(out,reply);
  if(op!="HELLO"&&op!="SESSION_OPEN"&&!emergency){cache[cacheIndex++%16]={id,request,reply,{}};}
  sendJson(Ack,reply);
}

void setup() {
  // Disable bridge before starting serial/tasks.
  digitalWrite(hw::STBY,LOW);pinMode(hw::STBY,OUTPUT);
  Serial.setTxBufferSize(4096);Serial.setRxBufferSize(4096);Serial.begin(460800);
  lock=xSemaphoreCreateMutex();streamQueue=xQueueCreate(64,sizeof(hw::Snapshot));eventQueue=xQueueCreate(16,sizeof(Notice));
  if(!lock||!streamQueue||!eventQueue){while(true){digitalWrite(hw::STBY,LOW);delay(100);}}
  hw::begin();configureAs5600();measureAdcRead();bootId=((uint64_t)esp_random()<<32)|esp_random();
  Preferences p;p.begin("furuta",true);String saved=p.getString("config","");uint32_t crc=p.getUInt("crc",0);p.end();
  if(saved.length()&&crc32((const uint8_t*)saved.c_str(),saved.length())==crc){
    DynamicJsonDocument d(4096);std::string why;Config c;
    if(!deserializeJson(d,saved)&&parseConfig(d.as<JsonObjectConst>(),c,why))control.cfg=c;
  }
  // A persisted profile never restores incremental position or arming.
  control.centered=false;control.state=State::Disarmed;currentFilter.reset(control.cfg);streamHz=control.cfg.streamHz;
  adcOversample=chooseOversample(control.cfg.adcHz);
  esp_task_wdt_init(1,true);
  xTaskCreatePinnedToCore(adcTask,"adc",4096,nullptr,8,&adcTaskHandle,0);
  xTaskCreatePinnedToCore(controlTask,"actuator",6144,nullptr,10,&controlTaskHandle,1);
  xTaskCreatePinnedToCore(pendulumTask,"pendulum",4096,nullptr,4,&pendTaskHandle,0);
  esp_timer_create_args_t a{};a.callback=adcTimerNotify;a.arg=&adcTaskHandle;a.name="adcTick";esp_timer_create(&a,&adcTimer);
  a.callback=timerNotify;
  a.arg=&controlTaskHandle;a.name="actuatorTick";esp_timer_create(&a,&controlTimer);
  a.arg=&pendTaskHandle;a.name="pendTick";esp_timer_create(&a,&pendTimer);
  esp_timer_start_periodic(adcTimer,1000000/control.cfg.adcHz);
  // Phase: control ticks at t0 + k*1 ms, pendulum reads at t0 + k*period - kPendLeadUs.
  const uint64_t t0=esp_timer_get_time();esp_timer_start_periodic(controlTimer,1000);
  while(esp_timer_get_time()<t0+1000-kPendLeadUs){}
  esp_timer_start_periodic(pendTimer,1000*kPendPeriodMs);
}
void loop() {
  static uint8_t rx[MAX_PACKET+16];static size_t used=0;static bool discard=false;
  for(int n=0;n<4096&&Serial.available();n++){
    uint8_t b=Serial.read();
    if(!b){if(!discard&&used){Packet p;if(decode(rx,used,p)&&p.type==Command)dispatch(p);}used=0;discard=false;}
    else if(!discard){if(used<sizeof(rx))rx[used++]=b;else{used=0;discard=true;}}
  }
  Notice e;
  if(xQueueReceive(eventQueue,&e,0)==pdTRUE){
    DynamicJsonDocument d(2048);d["event"]=e.text;
    xSemaphoreTake(lock,portMAX_DELAY);StatusCopy sc=statusCopy();
    double mean=calMean,sd=calN>1?sqrt(calM2/(calN-1)):0;uint32_t n=calN;
    xSemaphoreGive(lock);
    statusJson(d.createNestedObject("status"),sc);d["calMeanMv"]=mean;d["calSdMv"]=sd;d["calSamples"]=n;
    d["sensitivityMvA"]=control.cfg.sensitivityMvA;d["adcOversample"]=sc.adcOversample;std::string json;serializeJson(d,json);sendJson(Event,json);
  }
  static uint64_t lastTransmission=0;
  const uint64_t now=esp_timer_get_time();
  if(uxQueueMessagesWaiting(streamQueue)<4&&now-lastTransmission<20000){delay(1);return;}
  lastTransmission=now;
  std::vector<uint8_t> data;
  std::vector<hw::Snapshot> batch;hw::Snapshot s;
  while(batch.size()<16&&xQueuePeek(streamQueue,&s,0)==pdTRUE){
    if(!batch.empty()&&(s.configId!=batch[0].configId||s.runId!=batch[0].runId))break;
    if(xQueueReceive(streamQueue,&s,0)==pdTRUE)batch.push_back(s);
  }
  if(!batch.empty()){
    xSemaphoreTake(lock,portMAX_DELAY);put<uint64_t>(data,bootId);put<uint32_t>(data,batch[0].configId);put<uint32_t>(data,batch[0].runId);put<uint32_t>(data,dropped);xSemaphoreGive(lock);
    put<uint16_t>(data,batch.size());for(const auto&v:batch)hw::pack(data,v);sendPacket(Samples,data);
  }
  delay(1);
}
