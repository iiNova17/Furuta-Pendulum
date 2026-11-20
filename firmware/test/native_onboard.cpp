// Native test of the on-board controller (include/onboard.h).
//   g++ -std=c++17 -O2 -static -o native_onboard firmware/test/native_onboard.cpp && ./native_onboard firmware/test
//   (and once with -DONBOARD_OBSERVER_DOUBLE: the observer in double must equal Simulink to 1e-9)
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
#ifndef REPLAY_FLOAT_TOL
// The firmware runs the observer's model in float (ESP32-S3 FPU); Simulink in double. Measured 2026-11-20 on the replay
// runs: max |duty difference| 3.7e-6, no supervisor mismatch. 1e-4 keeps 25x headroom and is still 10x below one PWM
// step (1/1023). Build with -DONBOARD_OBSERVER_DOUBLE to check the same code against Simulink to 1e-9.
#define REPLAY_FLOAT_TOL 1e-4
#endif

static int replay(const std::string& file) {
  std::ifstream in(file);
  if(!in){std::printf("missing %s\n",file.c_str());return 1;}
  std::string line;std::getline(in,line);
  // The design's defaults, as Simulink ran them: breakaway feedforward (1.8) and balance compensation (1.9) on.
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
#ifdef ONBOARD_OBSERVER_DOUBLE
  const double tol=1e-9;     // observer model in double: the same arithmetic as Simulink, round-off only
#else
  const double tol=REPLAY_FLOAT_TOL;   // observer model in float (ESP32-S3 FPU): see the note at REPLAY_FLOAT_TOL
#endif
  return (n>100&&maxErr<tol&&modeErr==0)?0:1;
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
   // 1.12: the observer (default estKind 1) needs the 4 ms period; 1 / 2 ms run with the filtered differences (estKind 0)
   for(int i=0;i<rates::count;i++){assert(p.setPeriod(rates::Ts[i]));assert(p.valid(why)==(rates::Ts[i]>=.004-1e-9));
     OnboardParams q=p;q.estKind=0;assert(q.valid(why));for(int k=0;k<4;k++)assert(p.K[k]==rates::K[i][k]);}
   assert(p.setPeriod(.004));for(int k=0;k<4;k++)assert(std::abs(p.K[k]-design::K[k])<1e-9*std::abs(design::K[k]));
   assert(!p.setPeriod(.003));
   // a 1 ms controller from the same start settles like the 4 ms one (closed loop not run here: see onboard_rates.py)
   OnboardController c;c.p.setPeriod(.001);c.p.estKind=0;c.p.swingUp=false;c.reset();double u=c.step(0,.1);assert(c.mode==OnboardController::Stabilize&&u>0);}

  // 1.8 breakaway feedforward: from the hanging rest the kick must ask for more than the breakaway duty
  // (Rm tsp / Vs = 13 %); the 1.7 law (stickGain 0) asked for 11.1 % and the arm never moved on the rig.
  {OnboardController c;c.reset();const double u=c.step(0,3.14159265358979);
   OnboardController o;o.p.stickGain=0;o.reset();const double u0=o.step(0,3.14159265358979);
   const double breakaway=c.p.Rm*c.p.tsp/c.p.Vs;
   std::printf("kick from rest: %.1f %% duty with breakaway feedforward, %.1f %% without; breakaway %.1f %%\n",100*u,100*u0,100*breakaway);
   std::fflush(stdout);
   // with the 13 % breakaway of 2026-11-07 the 1.7 law stayed below it; with the 6 % of the 2026-11-20 model it is above
   // it already. What must hold: the feedforward kick clears the breakaway and is not smaller than the 1.7 law's.
   assert(c.mode==OnboardController::SwingUp&&u>breakaway+.02&&u>=u0);
   // moving fast, the feedforward is gone: same command as the 1.7 law
   // (filtered differences here: with the observer each controller's speed estimate depends on the duty it sent)
   OnboardController f;f.p.estKind=0;f.reset();f.step(0,2.);OnboardController f0;f0.p.estKind=0;f0.p.stickGain=0;f0.reset();f0.step(0,2.);
   const double uf=f.step(.02,2.05),uf0=f0.step(.02,2.05);assert(std::abs(uf-uf0)<1e-6);}

  // 1.9 balance compensation: a small LQR request with the arm at rest becomes at least the breakaway duty
  // 1.12: compensation is off by default (D.fric.gain 0, rig runs 2026-11-20); the law is tested with fricGain 1.
  {OnboardParams d0;assert(d0.fricGain==0);}
  {OnboardController c;c.p.swingUp=false;c.p.fricGain=1;c.reset();const double u=c.step(0,.02);   // pendulum 1.15 deg off upright (the smooth sign gives the full breakaway above ~2 % request)
   OnboardController o;o.p.swingUp=false;o.p.fricGain=0;o.reset();const double u0=o.step(0,.02);
   std::printf("balance, arm at rest, 1.15 deg lean: %.1f %% duty with compensation, %.1f %% without\n",100*u,100*u0);
   assert(c.mode==OnboardController::Stabilize&&u>c.p.Rm*c.p.tsp/c.p.Vs&&std::abs(u0)<.05);
   OnboardController k;k.p.swingUp=false;k.p.fricGain=1;k.p.fricStick=0;k.reset();const double uc=k.step(0,.02);   // Coulomb only
   // (13 % breakaway vs 7.4 % Coulomb on Table 6: Coulomb-only stayed below the breakaway; on the 20 Nov model the two
   // are about equal, 6.0 % vs 6.3 %.) Coulomb-only must still add most of the Coulomb duty over no compensation.
   std::printf("Coulomb only: %.1f %% duty\n",100*uc);std::fflush(stdout);
   assert(uc>u0+.8*c.p.fricDutyPos);}

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
  // 1.10: currentChecks false ignores the current sensor (clipped ADC reading, overcurrent), nothing else
  {Controller q;std::string w;q.cfg.maxDuty=.5;q.cfg.travelCounts=500;q.cfg.slewPerSecond=100;q.cfg.currentZeroMv=2500;q.cfg.maxDuration=0;
   q.centered=true;assert(q.arm(20,1000000,w));assert(q.startExternal(1000000,Controller::Source::Onboard,w));
   q.tick(1001000,0,0,3150,100,true);assert(q.fault=="ADC_INVALID_OR_CLIPPED");            // checks on: clipped reading trips
   q.state=State::Disarmed;q.fault.clear();q.cfg.currentChecks=false;q.cfg.currentZeroMv=NAN;assert(q.arm(21,2000000,w)); // no zero needed
   assert(q.startExternal(2000000,Controller::Source::Onboard,w));q.external=.3;
   for(int i=1;i<=30;i++){q.lastHeartbeat=2000000+i*1000;q.tick(2000000+i*1000,0,0,3150,100,true);}   // 3150 mV, 10 A
   assert(q.state==State::Running&&q.applied>0);
   q.tick(2000000+31*1000,600,0,3150,100,true);assert(q.fault=="ARM_TRAVEL");}               // other trips stay

  // 1.8: maxDuration 0 = no run-length limit; every trip still applies
  {Config c=k.cfg;c.maxDuration=0;std::string w;assert(c.valid(w));c.maxDuration=-1;assert(!c.valid(w));}
  k.cfg.maxDuration=0;assert(k.arm(12,7000000,why));assert(k.startExternal(7000000,Controller::Source::Onboard,why));
  for(int i=1;i<=3600;i++){k.lastHeartbeat=7000000+(uint64_t)i*1000000;k.tick(7000000+(uint64_t)i*1000000,0,0,2500,100,true);}
  assert(k.state==State::Running);                                                        // an hour later: still running
  k.tick(7000000+3600ull*1000000+400000,0,0,2500,100,true);assert(k.fault=="HEARTBEAT_TIMEOUT"); // heartbeat still ends it

  std::printf(fails?"FAILED\n":"native_onboard OK\n");
  return fails;
}
