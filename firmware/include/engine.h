#pragma once
#include <stdint.h>
#include <cmath>
#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace fw {
constexpr double kPi=3.14159265358979323846;
constexpr size_t kMaxSegments=256;
enum class State:uint8_t { Boot,Disarmed,Armed,Running,Stopping,Fault };
inline const char* stateName(State s) {
  static const char* names[]={"BOOT","DISARMED","ARMED_IDLE","RUNNING","STOPPING","FAULT"};
  return names[(unsigned)s];
}
struct Config {
  double maxDuty=0,slewPerSecond=.5,maxDuration=10,travelCounts=0,velocityCountsPerSecond=0;
  double currentTripA=4,currentTripMs=10,currentZeroMv=NAN,sensitivityMvA=185,countsPerRev=NAN;
  int armSign=1,pendSign=1,pendDownRaw=-1;
  // Measured breakaway duty (fraction, NaN = unknown). Stored only; not applied to commands.
  double deadzonePos=NAN,deadzoneNeg=NAN;
  double assumedBusV=12;uint32_t heartbeatMs=300,streamHz=250,adcHz=2000,pwmHz=20000;
  std::string filterType="bypass"; double alpha=.1;unsigned window=8;std::array<double,6> sos{{1,0,0,1,0,0}};
  bool valid(std::string& why)const {
    if(!std::isfinite(maxDuty)||maxDuty<0||maxDuty>1||!std::isfinite(slewPerSecond)||slewPerSecond<=0||slewPerSecond>100 ||
       !std::isfinite(maxDuration)||maxDuration<0||maxDuration>3600 ||      // 0 = no run-length limit (1.8)
       !std::isfinite(travelCounts)||travelCounts<0||travelCounts>1e9||
       !std::isfinite(velocityCountsPerSecond)||velocityCountsPerSecond<0||
       !std::isfinite(currentTripA)||currentTripA<0||currentTripA>50||
       !std::isfinite(currentTripMs)||currentTripMs<0||currentTripMs>1000||
       !std::isfinite(sensitivityMvA)||sensitivityMvA<=0||
       !(std::isnan(currentZeroMv)||(currentZeroMv>=1800&&currentZeroMv<=3000))||
       !(std::isnan(countsPerRev)||(countsPerRev>0&&std::isfinite(countsPerRev)))||
       !(std::isnan(deadzonePos)||(deadzonePos>=0&&deadzonePos<=1))||!(std::isnan(deadzoneNeg)||(deadzoneNeg>=0&&deadzoneNeg<=1))||
       (armSign!=1&&armSign!=-1)||(pendSign!=1&&pendSign!=-1)||pendDownRaw < -1||pendDownRaw>4095||
       !std::isfinite(assumedBusV)||assumedBusV<=0||assumedBusV>13.5||
       heartbeatMs<100||heartbeatMs>2000||(streamHz!=50&&streamHz!=100&&streamHz!=250)||
       adcHz<100||adcHz>2000||1000000%adcHz!=0||pwmHz<1000||pwmHz>30000) {
      why="Invalid profile value/rate";return false;
    }
    if(filterType!="bypass"&&filterType!="ema"&&filterType!="moving"&&filterType!="biquad") {
      why="Unsupported causal filter";return false;
    }
    if(!std::isfinite(alpha)||alpha<=0||alpha>1||window<1||window>256) {why="Invalid filter parameters";return false;}
    for(double x:sos)if(!std::isfinite(x)){why="Nonfinite SOS";return false;}
    if(sos[3]!=1 || std::abs(sos[5])>=1 || 1+sos[4]+sos[5]<=0 || 1-sos[4]+sos[5]<=0) {
      why="SOS denominator must be normalized and strictly stable";return false;
    }
    return true;
  }
};
inline double wrap(double x) {return x-2*kPi*std::floor((x+kPi)/(2*kPi));}
inline int deltaAngle(int x,int prev) {int d=x-prev;if(d>2047)d-=4096;if(d< -2048)d+=4096;return d;}
inline uint16_t prbs(uint16_t seed,uint64_t index) {
  uint16_t x=(seed&511);if(!x)x=1;
  for(uint64_t i=0;i<index%511;i++) x=(x>>1)|(((x^(x>>4))&1)<<8);
  return x&1;
}
struct Segment {
  std::string kind="hold";double duration=1,amplitude=0,offset=0,f0=1,f1=2,phase=0,duty=.5;
  double end=0,bitPeriod=.05;uint16_t seed=1;bool logarithmic=false,linear=false;
  std::vector<double> frequencies,amplitudes,phases,table;double tablePeriod=.01;
  double value(double t)const {
    t=std::max(0.,t); double v=0,p=2*kPi*f0*t+phase;
    if(kind=="hold"||kind=="step")v=amplitude;
    else if(kind=="pulse")v=t<duty*duration?amplitude:0;
    else if(kind=="doublet")v=t<duration*.5?amplitude:-amplitude;
    else if(kind=="sine")v=amplitude*std::sin(p);
    else if(kind=="square"){double z=f0*t+phase/(2*kPi);z-=std::floor(z);v=amplitude*(z<duty?1:-1);}
    else if(kind=="triangle") {double z=f0*t+phase/(2*kPi);z-=std::floor(z);v=amplitude*(1-4*std::abs(z-.5));}
    else if(kind=="ramp")v=amplitude+(end-amplitude)*std::min(1.,t/duration);
    else if(kind=="chirp") {
      double cycles=f0*t+(f1-f0)*t*t/(2*duration);
      if(logarithmic && f0!=f1)cycles=f0*duration/std::log(f1/f0)*(std::pow(f1/f0,t/duration)-1);
      v=amplitude*std::sin(2*kPi*cycles+phase);
    } else if(kind=="prbs")v=amplitude*(prbs(seed,(uint64_t)std::floor(t/bitPeriod))?1:-1);
    else if(kind=="multisine") {
      for(size_t i=0;i<frequencies.size();i++)v+=amplitudes[i]*std::sin(2*kPi*frequencies[i]*t+phases[i]);
    } else if(kind=="arbitrary" && !table.empty()) {
      double index=t/tablePeriod;size_t i=std::min((size_t)index,table.size()-1);v=table[i];
      if(linear && i+1<table.size())v+=(table[i+1]-v)*(index-i);
    }
    return offset+v;
  }
  bool validate(const Config& cfg,std::string& why)const {
    const std::string allowed=" hold step pulse doublet sine square triangle ramp chirp prbs multisine arbitrary ";
    if(allowed.find(" "+kind+" ")==std::string::npos){why="Unknown waveform";return false;}
    for(double x:{duration,amplitude,offset,f0,f1,phase,duty,end,bitPeriod,tablePeriod})
      if(!std::isfinite(x)){why="Nonfinite waveform";return false;}
    if(duration<=0||(cfg.maxDuration>0&&duration>cfg.maxDuration)||f0<=0||f1<=0||f0>=500||f1>=500||
       duty<=0||duty>1||bitPeriod<.002||tablePeriod<.001){why="Waveform timing out of range";return false;}
    double peak=std::abs(amplitude);
    if(kind=="ramp")peak=std::max(peak,std::abs(end));
    if(kind=="multisine") {
      if(frequencies.empty()||frequencies.size()>8||frequencies.size()!=amplitudes.size()||phases.size()!=amplitudes.size()){why="Multisine vectors differ";return false;}
      peak=0;for(size_t i=0;i<frequencies.size();i++){
        if(!std::isfinite(frequencies[i])||frequencies[i]<=0||frequencies[i]>=500||!std::isfinite(amplitudes[i])||!std::isfinite(phases[i])){why="Invalid tone";return false;}
        peak+=std::abs(amplitudes[i]);
      }
    }
    if(kind=="arbitrary") {
      if(table.empty()||table.size()>4096||duration>tablePeriod*table.size()){why="Table length/duration invalid";return false;}
      peak=0;for(double x:table){if(!std::isfinite(x)){why="Nonfinite table";return false;}peak=std::max(peak,std::abs(x));}
    }
    if(std::abs(offset)+peak>cfg.maxDuty+1e-9){why="Amplitude plus offset exceeds profile";return false;}
    return true;
  }
};
struct Experiment {
  std::vector<Segment> segments;
  double duration()const {double t=0;for(const auto&s:segments)t+=s.duration;return t;}
  bool validate(const Config& c,std::string&w)const {
    if(segments.empty()||segments.size()>kMaxSegments||(c.maxDuration>0&&duration()>c.maxDuration)){w="Experiment length/segments exceed limits";return false;}
    for(const auto&s:segments)if(!s.validate(c,w))return false;
    return true;
  }
  double value(double t)const {
    if(t<0)return 0;
    for(const auto&s:segments){if(t<s.duration)return s.value(t);t-=s.duration;}
    return 0;
  }
};
struct Filter {
  Config cfg;std::array<double,256> ring{};unsigned index=0,count=0;double sum=0,y=0,z1=0,z2=0;bool started=false;
  void reset(const Config& c){cfg=c;ring.fill(0);index=count=0;sum=y=z1=z2=0;started=false;}
  double step(double x) {
    if(!started){y=x;started=true;}
    if(cfg.filterType=="ema"){y+=cfg.alpha*(x-y);return y;}
    if(cfg.filterType=="moving"){sum-=ring[index];ring[index]=x;sum+=x;index=(index+1)%cfg.window;count=std::min(count+1,cfg.window);return sum/count;}
    if(cfg.filterType=="biquad"){double v=cfg.sos[0]*x+z1;z1=cfg.sos[1]*x-cfg.sos[4]*v+z2;z2=cfg.sos[2]*x-cfg.sos[5]*v;return v;}
    return x;
  }
};
struct Controller {
  // Where a run's duty request comes from: an uploaded experiment, or a closed-loop controller
  // (Onboard: include/onboard.h, set every controller period through `external`). The safety
  // checks, clip, slew and reversal logic below are the same for every source.
  enum class Source:uint8_t { Experiment,Onboard };
  State state=State::Disarmed;Config cfg;Experiment experiment;Source source=Source::Experiment;double external=0;
  int64_t center=0;bool centered=false;uint64_t session=0,lastHeartbeat=0,started=0,lastTick=0,tripSince=0,reverseUntil=0;
  uint32_t run=0;double requested=0,applied=0;std::string fault;uint32_t flags=0;
  void stop(){applied=requested=0;if(state!=State::Fault)state=State::Disarmed;session=0;reverseUntil=0;}
  void fail(const char*s){applied=requested=0;state=State::Fault;fault=s;session=0;}
  bool arm(uint64_t s,uint64_t now,std::string&w) {
    if(state!=State::Disarmed||!fault.empty()||!centered||!s||cfg.maxDuty<=0||cfg.travelCounts<=0){w="Disarmed state, cleared fault, centered arm and commissioned limits required";return false;}
    if(cfg.currentTripA>0&&!std::isfinite(cfg.currentZeroMv)){w="Current trip requires zero calibration";return false;}
    if(!cfg.valid(w))return false;
    session=s;lastHeartbeat=now;state=State::Armed;return true;
  }
  bool start(uint64_t now,std::string&w) {
    if(state!=State::Armed){w="ARM required";return false;}
    if(!experiment.validate(cfg,w))return false;
    source=Source::Experiment;started=lastTick=now;tripSince=0;run++;state=State::Running;return true;
  }
  // Closed-loop run: the duty request is `external`; the run ends after cfg.maxDuration.
  bool startExternal(uint64_t now,Source s,std::string&w) {
    if(state!=State::Armed){w="ARM required";return false;}
    source=s;external=0;started=lastTick=now;tripSince=0;run++;state=State::Running;return true;
  }
  // mv is the latest (oversampled) conversion, used for clip/validity checks.
  // tripMv is the MCU-filtered current used for the overcurrent trip; when it
  // is not finite (filter not started) the trip falls back to mv.
  void tick(uint64_t now,int64_t count,double cps,double mv,uint64_t adcAge,bool encoderOk=true,double tripMv=NAN) {
    flags=0;if(state!=State::Armed&&state!=State::Running)return;
    if(now-lastHeartbeat>cfg.heartbeatMs*1000ULL){fail("HEARTBEAT_TIMEOUT");return;}
    if(!encoderOk){fail("ENCODER_INVALID");return;}
    if(std::abs((double)(count-center))>cfg.travelCounts){fail("ARM_TRAVEL");return;}
    if(cfg.velocityCountsPerSecond>0&&std::abs(cps)>cfg.velocityCountsPerSecond){fail("ARM_VELOCITY");return;}
    if(adcAge>20000||!std::isfinite(mv)||mv<100||mv>3100){fail("ADC_INVALID_OR_CLIPPED");return;}
    const double tm=std::isfinite(tripMv)?tripMv:mv;
    if(cfg.currentTripA>0&&std::abs((tm-cfg.currentZeroMv)/cfg.sensitivityMvA)>cfg.currentTripA) {
      if(!tripSince)tripSince=now;
      if(now-tripSince>=cfg.currentTripMs*1000){fail("OVERCURRENT");return;}
    }else tripSince=0;
    if(state!=State::Running)return;
    // Experiments end with their sequence; closed-loop runs at maxDuration, or never when it is 0 (heartbeat,
    // STOP and every trip still apply).
    const double length=source==Source::Experiment?experiment.duration():cfg.maxDuration;
    if((source==Source::Experiment||cfg.maxDuration>0)&&(now-started)*1e-6>=length){stop();return;}
    requested=source==Source::Experiment?experiment.value((now-started)*1e-6):(std::isfinite(external)?external:0.);
    double target=std::max(-cfg.maxDuty,std::min(cfg.maxDuty,requested));
    if(target!=requested)flags|=1;
    double dt=std::min(.01,(now-lastTick)*1e-6);lastTick=now;
    double rate=cfg.slewPerSecond*dt;
    // Go to zero before reversing. One millisecond standby blanking follows.
    if(applied*target<0)target=0;
    double next=applied+std::max(-rate,std::min(rate,target-applied));
    if(std::abs(next)<1e-9&&std::abs(applied)>1e-9){next=0;reverseUntil=now+1000;}
    if(now<reverseUntil){next=0;flags|=4;}
    if(std::abs(next-target)>1e-8)flags|=2;
    applied=next;
  }
};
}
