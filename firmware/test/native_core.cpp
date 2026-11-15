#include "../../firmware/include/engine.h"
#include "../../firmware/include/protocol.h"
#include "../../firmware/include/records.h"
#include <cassert>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
using namespace fw;
std::string hex(const std::vector<uint8_t>&v){std::ostringstream s;for(auto b:v)s<<std::hex<<std::setfill('0')<<std::setw(2)<<(int)b;return s.str();}
int main(int argc,char**argv) {
  const uint8_t reference[]={'1','2','3','4','5','6','7','8','9'};assert(crc32(reference,9)==0xcbf43926);
  for(size_t n=0;n<2040;n+=17){
    std::vector<uint8_t> raw(n);for(size_t i=0;i<n;i++)raw[i]=(i*37)%256;
    auto encoded=cobs(raw);std::vector<uint8_t> result;
    assert(uncobs(encoded.data(),encoded.size()-1,result));assert(raw==result);
  }
  Packet p;p.type=Command;p.session=0x123456789abcdef0ULL;p.sequence=19;p.payload={0,1,2,0,255};
  auto bytes=encode(p);Packet q;assert(decode(bytes.data(),bytes.size()-1,q));assert(q.payload==p.payload&&q.session==p.session);
  bytes[2]^=32;assert(!decode(bytes.data(),bytes.size()-1,q));
  Config cfg;std::string why;assert(cfg.valid(why));Controller c;assert(!c.arm(4,1000,why));
  cfg.maxDuty=.2;cfg.travelCounts=5000;cfg.slewPerSecond=1;cfg.currentZeroMv=2500;
  c.cfg=cfg;c.centered=true;assert(c.arm(4,1000000,why));
  Segment s;s.kind="hold";s.duration=.1;s.amplitude=.1;c.experiment.segments={s};assert(c.start(1000000,why));
  c.tick(1001000,0,0,2500,100,true);assert(c.applied>0&&c.applied<=.00101);
  c.tick(1101000,0,0,2500,100,true);assert(c.state==State::Disarmed&&c.applied==0);
  assert(c.arm(4,2000000,why));c.tick(2301000,0,0,2500,100,true);assert(c.state==State::Fault&&c.fault=="HEARTBEAT_TIMEOUT");
  c.stop();assert(c.state==State::Fault);assert(!c.arm(5,3000000,why));
  c.state=State::Disarmed;c.fault.clear();assert(c.arm(5,3000000,why));c.tick(3001000,6000,0,2500,100,true);assert(c.fault=="ARM_TRAVEL");
  c.state=State::Disarmed;c.fault.clear();assert(c.arm(5,4000000,why));c.tick(4001000,0,0,2500,21000,true);assert(c.fault=="ADC_INVALID_OR_CLIPPED");
  // Overcurrent trip uses the filtered value: a raw spike alone must not trip.
  c.state=State::Disarmed;c.fault.clear();c.cfg.currentTripA=1;c.cfg.currentTripMs=0;c.cfg.sensitivityMvA=185;
  assert(c.arm(6,5000000,why));c.tick(5001000,0,0,2500+185*2,100,true,2500);assert(c.state==State::Armed);
  c.tick(5002000,0,0,2500,100,true,2500+185*1.5);assert(c.fault=="OVERCURRENT");
  c.state=State::Disarmed;c.fault.clear();assert(c.arm(7,6000000,why));c.tick(6001000,0,0,2500+185*1.5,100,true);assert(c.fault=="OVERCURRENT");
  c.cfg.currentTripA=0;
  {Config z;z.deadzonePos=.14;z.deadzoneNeg=.16;assert(z.valid(why));z.deadzonePos=1.5;assert(!z.valid(why));}
  // Raised caps (firmware 1.3): long runs, 256 segments, 450 Hz waveforms, 40 A trip setting.
  {Config big;big.maxDuty=1;big.maxDuration=3000;big.currentTripA=40;assert(big.valid(why));
   Experiment e;Segment q;q.kind="sine";q.amplitude=1;q.f0=450;q.duration=1;e.segments.assign(256,q);assert(e.validate(big,why));
   e.segments.push_back(q);assert(!e.validate(big,why));}
  for(int i=0;i<4096;i++){assert(deltaAngle((i+1)%4096,i)==1);assert(deltaAngle((i+4095)%4096,i)==-1);}
  assert(deltaAngle(2048,0)==-2048);
  Config fc;fc.filterType="moving";fc.window=3;Filter f;f.reset(fc);assert(f.step(1)==1);assert(f.step(3)==2);assert(f.step(5)==3);assert(f.step(7)==5);
  fc.filterType="ema";fc.alpha=.5;f.reset(fc);assert(f.step(2)==2);assert(f.step(4)==3);
  fc.filterType="biquad";fc.sos={1,0,0,1,0,1.1};assert(!fc.valid(why));
  Segment square;square.kind="square";square.amplitude=.2;square.phase=-.2;assert(square.value(0)==-.2);
  Segment ramp;ramp.kind="ramp";ramp.amplitude=-.1;ramp.end=.1;ramp.duration=2;assert(std::abs(ramp.value(1))<1e-10);
  hw::Snapshot record;record.t=0xf123456789abcdefULL;record.seq=42;record.count=-9007199254740993LL;
  record.pend=4095;record.adc=3200;record.mv=2500.5;record.filtered=2499.25;record.status=48;record.agc=128;
  record.mag=1700;record.flags=15;record.requested=-.125;record.applied=.0625;
  record.adcTime=record.t-20;record.pendTime=record.t-100;record.applyTime=record.t-10;
  std::vector<uint8_t> core;put<uint64_t>(core,0x123456789abcdef0ULL);put<uint32_t>(core,3);put<uint32_t>(core,2);
  put<uint32_t>(core,0);put<uint16_t>(core,1);hw::pack(core,record);assert(core.size()==110);
  if(argc>1){
    std::ofstream out(argv[1]);out<<"{\n \"crc123456789\":\"cbf43926\",\n \"vectors\":[\n";
    for(int i=0;i<4;i++){
      Packet v;v.type=i+1;v.session=0x123456789abcdef0ULL;v.sequence=i+1;
      v.payload=i==0?std::vector<uint8_t>{}:i==1?std::vector<uint8_t>{0,1,0,255}:std::vector<uint8_t>(i==2?254:1024,0x37);
      out<<"{\"type\":"<<(int)v.type<<",\"sequence\":"<<v.sequence<<",\"payloadHex\":\""<<hex(v.payload)<<"\",\"frameHex\":\""<<hex(encode(v))<<"\"}"<<(i<3?",":"")<<"\n";
    }
    out<<"],\n \"corePayloadHex\":\""<<hex(core)<<"\",\n \"coreTime\":\""<<record.t<<"\",\n \"waveforms\":[\n";int i=0;
    for(const std::string kind:{"sine","square","triangle","chirp","prbs","ramp","doublet","pulse"}){
      Segment a;a.kind=kind;a.amplitude=.2;a.offset=.01;a.duration=2;a.f0=1;a.f1=3;a.phase=-.2;a.end=-.1;
      out<<"{\"kind\":\""<<kind<<"\",\"values\":[";
      for(int j=0;j<20;j++)out<<std::setprecision(16)<<a.value(j*.073)<<(j<19?",":"");
      out<<"]}"<<(++i<8?",":"")<<"\n";
    }
    out<<"]\n}\n";
  }
  puts("PASS: COBS/CRC/frames, angle wrap, waveform, filters, commissioned arm, completion and faults.");
}
