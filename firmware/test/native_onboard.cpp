// Native test of the on-board controller (include/onboard.h).
//   g++ -std=c++17 -O2 -o native_onboard firmware/test/native_onboard.cpp && ./native_onboard firmware/test
// 1. Replays harness runs (onboard_replay_*.csv, written by export_onboard.m): the measured angles that
//    the Simulink controller received go through the C++ copy, which must command the same duty and be in
//    the same supervisor state at every 4 ms period.
// 2. Checks the device-reading conversions and the run-control integration in engine.h.
#include "../include/onboard.h"
#include "../include/engine.h"
#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
using namespace fw;

static int replay(const std::string& file) {
  std::ifstream in(file);
  if(!in){std::printf("missing %s\n",file.c_str());return 1;}
  std::string line;std::getline(in,line);
  OnboardController c;c.reset();
  double maxErr=0;size_t n=0,modeErr=0,firstBad=0;
  while(std::getline(in,line)) {
    double t,th0,th1,r,duty;int mode;
    if(std::sscanf(line.c_str(),"%lf,%lf,%lf,%lf,%lf,%d",&t,&th0,&th1,&r,&duty,&mode)!=6)continue;
    c.p.r=r;
    double u=c.step(th0,th1);
    double err=std::abs(u-duty);
    if(err>maxErr)maxErr=err;
    if((int)c.mode!=mode){if(!modeErr)firstBad=n;modeErr++;}
    n++;
  }
  std::printf("%s: %zu periods, max |duty error| %.3g, supervisor state mismatches %zu (first at %zu)\n",
              file.c_str(),n,maxErr,modeErr,firstBad);
  return (n>100&&maxErr<1e-9&&modeErr==0)?0:1;
}

int main(int argc,char**argv) {
  std::string dir=argc>1?argv[1]:".";
  int fails=0;
  fails+=replay(dir+"/onboard_replay_swingup.csv");
  fails+=replay(dir+"/onboard_replay_push.csv");

  // conversions: hanging reads +-pi, upright 0, a quarter turn of the arm is pi/2
  assert(std::abs(std::abs(pendAngle(2810,2810,-1))-3.14159265358979)<1e-9);
  assert(std::abs(pendAngle((2810+2048)%4096,2810,-1))<1e-9);
  assert(std::abs(armAngle(374,0,1,1496)-3.14159265358979/2)<1e-9);
  assert(std::abs(armAngle(-374,0,-1,1496)-3.14159265358979/2)<1e-9);
  {OnboardParams p;std::string why;assert(p.valid(why));p.Ts=.003;assert(!p.valid(why));
   // 1.7: 1 / 2 / 4 ms periods, each with the design's LQR gain (onboard_rates.h); 4 ms = design::K
   for(int i=0;i<rates::count;i++){assert(p.setPeriod(rates::Ts[i])&&p.valid(why));for(int k=0;k<4;k++)assert(p.K[k]==rates::K[i][k]);}
   assert(p.setPeriod(.004));for(int k=0;k<4;k++)assert(std::abs(p.K[k]-design::K[k])<1e-9*std::abs(design::K[k]));
   assert(!p.setPeriod(.003));
   // a 1 ms controller from the same start settles like the 4 ms one (closed loop not run here: see onboard_rates.py)
   OnboardController c;c.p.setPeriod(.001);c.p.swingUp=false;c.reset();double u=c.step(0,.1);assert(c.mode==OnboardController::Stabilize&&u>0);}

  // run control: an onboard run takes the external request, keeps every safety check, ends at maxDuration
  Controller k;std::string why;
  k.cfg.maxDuty=.5;k.cfg.travelCounts=500;k.cfg.slewPerSecond=100;k.cfg.currentZeroMv=2500;k.cfg.maxDuration=2;
  k.centered=true;assert(k.arm(9,1000000,why));
  assert(k.startExternal(1000000,Controller::Source::Onboard,why));
  k.external=.3;k.tick(1001000,0,0,2500,100,true);assert(k.state==State::Running&&k.applied>0&&k.applied<=.1+1e-9);
  k.external=.9;for(int i=2;i<20;i++){k.lastHeartbeat=1000000+i*1000;k.tick(1000000+i*1000,0,0,2500,100,true);}
  assert(std::abs(k.applied-.5)<1e-9);                       // clipped to maxDuty
  k.lastHeartbeat=3000000;k.tick(3000001,0,0,2500,100,true);assert(k.state==State::Disarmed);   // maxDuration
  assert(k.arm(10,4000000,why));assert(k.startExternal(4000000,Controller::Source::Onboard,why));
  k.tick(4001000,600,0,2500,100,true);assert(k.fault=="ARM_TRAVEL"&&k.applied==0);        // travel trip still active
  k.state=State::Disarmed;k.fault.clear();assert(k.arm(11,5000000,why));
  assert(k.startExternal(5000000,Controller::Source::Onboard,why));
  k.tick(5400000,0,0,2500,100,true);assert(k.fault=="HEARTBEAT_TIMEOUT");                  // heartbeat still active
  k.state=State::Disarmed;k.fault.clear();assert(!k.startExternal(6000000,Controller::Source::Onboard,why)); // needs ARM

  std::printf(fails?"FAILED\n":"native_onboard OK\n");
  return fails;
}
