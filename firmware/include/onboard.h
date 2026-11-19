#pragma once
// On-board copy of the hybrid controller (matlab/harness/controllers/hybrid/ctrl_hybrid.slx):
// estimator -> LQR (+ friction compensation) and energy swing-up -> Stateflow supervisor.
// Pure C++ (no Arduino) so the native test can replay harness runs through it and require the
// same duty as Simulink (test/native_onboard.cpp). Numbers come from onboard_design.h, which
// export_onboard.m generates from ctrl_hybrid_design.m. The firmware safety path (clip, slew,
// travel, current, heartbeat) is applied after this, exactly as for host-driven experiments.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include "onboard_design.h"

namespace fw {
struct OnboardParams {
  double Ts=design::Ts, velFc=design::velFc;
  double K[4]={design::K[0],design::K[1],design::K[2],design::K[3]};
  double fricGain=design::fricGain, fricBand=design::fricBand, fricDutyPos=design::fricDutyPos, fricDutyNeg=design::fricDutyNeg;
  double catchTh1=design::catchTh1, catchW1=design::catchW1, exitTh1=design::exitTh1, safeTh0=design::safeTh0;
  bool rearm=design::rearm!=0;
  double E0=design::sw_E0, kE=design::sw_k_E, aPush=design::sw_a_push, wc=design::sw_wc, zc=design::sw_zc;
  double aKick=design::sw_a_kick, kickE=design::sw_kick_E, wSign=design::sw_w_sign, wFric=design::sw_w_fric;
  double iMax=design::sw_i_max, uMax=design::sw_u_max;
  double Jp=design::pl_Jp, G=design::pl_G, Ja=design::pl_Ja, Beq=design::pl_Beq, tcp=design::pl_tcp, tcn=design::pl_tcn;
  double Rm=design::pl_Rm, Kb=design::pl_Kb, Vs=design::pl_Vs;
  double r=0;               // arm reference [rad]
  bool swingUp=true;        // false: balance only (start near upright); losing the pendulum ends the run
  bool valid(std::string& why) const {
    const double all[]={Ts,velFc,K[0],K[1],K[2],K[3],fricGain,fricBand,fricDutyPos,fricDutyNeg,catchTh1,catchW1,exitTh1,
      safeTh0,E0,kE,aPush,wc,zc,aKick,kickE,wSign,wFric,iMax,uMax,Jp,G,Ja,Beq,tcp,tcn,Rm,Kb,Vs,r};
    for(double x:all) if(!std::isfinite(x)){why="Nonfinite onboard parameter";return false;}
    if(std::abs(Ts-.004)>1e-9){why="Onboard controller runs at 4 ms (the period it was designed for)";return false;}
    if(velFc<=0||fricBand<=0||wSign<=0||wFric<=0||Jp<=0||G<=0||Ja<=0||Rm<=0||Vs<=0||iMax<=0||uMax<=0||uMax>1||
       catchTh1<=0||exitTh1<catchTh1||safeTh0<=0){why="Onboard parameter out of range";return false;}
    return true;
  }
};

// Device readings -> model angles, with the same conventions as the lab app (signals.py,
// estimation.py): theta0 positive in the armSign direction from the marked centre; theta1 = 0
// upright, +-pi hanging, from the AS5600 raw value and the pendulum-down reference.
inline double armAngle(long long count,long long center,int armSign,double countsPerRev) {
  return armSign*double(count-center)*2*3.14159265358979323846/countsPerRev;
}
inline double pendAngle(int raw,int downRaw,int pendSign) {
  const double pi=3.14159265358979323846;
  double a=pendSign*double(raw-downRaw)*2*pi/4096+pi;
  return a-2*pi*std::floor((a+pi)/(2*pi));
}

struct OnboardController {
  enum Mode:uint8_t {SafeStop=0,SwingUp=1,Stabilize=2};
  OnboardParams p;
  bool started=false;Mode mode=SwingUp;
  double prev[2]{0,0},w[2]{0,0},x[4]{0,0,0,0},E=0,uStab=0,uSwing=0,duty=0,alpha=0;
  // reset() before every run, after setting p (the filter pole is computed once here)
  void reset(){started=false;mode=SwingUp;prev[0]=prev[1]=w[0]=w[1]=0;for(double&v:x)v=0;E=uStab=uSwing=duty=0;
    alpha=std::exp(-2*3.14159265358979323846*p.velFc*p.Ts);}
  static double sat(double v,double lim){return std::min(lim,std::max(-lim,v));}
  static double wrapPi(double a){const double pi=3.14159265358979323846;return a-2*pi*std::floor((a+pi)/(2*pi));}

  // estimator block: wrap theta1, filtered differences (ctrl_hybrid.slx / estimator)
  void estimate(double th0,double th1) {
    double th[2]={th0,wrapPi(th1)};
    if(!started){prev[0]=th[0];prev[1]=th[1];w[0]=w[1]=0;started=true;}
    const double a=alpha;
    double d0=th[0]-prev[0],d1=wrapPi(th[1]-prev[1]);
    w[0]=a*w[0]+(1-a)*d0/p.Ts;w[1]=a*w[1]+(1-a)*d1/p.Ts;
    prev[0]=th[0];prev[1]=th[1];
    x[0]=th[0];x[1]=th[1];x[2]=w[0];x[3]=w[1];
  }
  // LQR block + friction compensation block
  double lqr() const {
    double u0=-(p.K[0]*(x[0]-p.r)+p.K[1]*x[1]+p.K[2]*x[2]+p.K[3]*x[3]);
    double s=std::tanh(u0/p.fricBand);
    return u0+p.fricGain*(p.fricDutyPos*std::max(s,0.)+p.fricDutyNeg*std::min(s,0.));
  }
  // swingup_energy.slx / energy law (report section 10)
  double energyLaw() {
    const double th0=x[0],th1=x[1],w0=x[2],w1=x[3];
    E=.5*p.Jp*w1*w1+p.G*(std::cos(th1)-1);
    const double e=E/(2*p.G);
    double a0=-p.aPush*std::min(1.,std::max(-1.,p.kE*(p.E0-e)))*std::tanh(w1*std::cos(th1)/p.wSign);
    if(e< -1+p.kickE)a0=p.aKick;
    a0=a0-p.wc*p.wc*(th0-p.r)-2*p.zc*p.wc*w0;
    const double sf=std::tanh(w0/p.wFric);
    double i=p.Ja*a0+p.Beq*w0+p.tcp*std::max(sf,0.)+p.tcn*std::min(sf,0.);
    i=std::min(p.iMax,std::max(-p.iMax,i));
    double u=(p.Rm*i+p.Kb*w0)/p.Vs;
    return std::min(p.uMax,std::max(-p.uMax,u));
  }
  // Stateflow supervisor: transitions first (SafeStop has priority 1), then the state's action.
  void supervise(bool first) {
    const double th0=x[0],th1=x[1],w1=x[3];
    const bool catchOk=std::abs(th1)<p.catchTh1&&std::abs(w1)<p.catchW1;
    if(first){mode=catchOk?Stabilize:SwingUp;return;}
    if(mode==SwingUp){if(std::abs(th0)>p.safeTh0)mode=SafeStop;else if(catchOk)mode=Stabilize;}
    else if(mode==Stabilize){if(std::abs(th0)>p.safeTh0)mode=SafeStop;else if(std::abs(th1)>p.exitTh1)mode=SwingUp;}
    else if(p.rearm&&std::abs(th0)<p.safeTh0/2&&std::abs(th1)>3.14159265358979323846-.2&&std::abs(w1)<.5)mode=SwingUp;
  }
  // One controller period: measured arm angle and pendulum angle [rad] -> duty request.
  double step(double th0,double th1) {
    const bool first=!started;
    estimate(th0,th1);
    uStab=lqr();uSwing=energyLaw();
    supervise(first);
    duty=mode==Stabilize?uStab:(mode==SwingUp?uSwing:0.);
    return duty;
  }
};
}
