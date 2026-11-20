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
#include "onboard_rates.h"

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
  // 1.8: arm breakaway friction (measured dead zone) and how the swing-up's motor inverse uses it
  double tsp=design::pl_tsp, tsn=design::pl_tsn, stickGain=design::sw_stick_gain, wStick=design::sw_w_stick, aStick=design::sw_a_stick;
  double fricStick=design::fricStick;   // 1.9: breakaway part of the balance friction compensation (0 = 1.8)
  // 1.12: model-assisted observer (est_observer.slx / est_observer_ref.m). The pendulum reading lags the arm encoder
  // by 9-15 ms on the rig (shaft coupling, measured 2026-11-20); the observer estimates the state at the reading's
  // time and predicts it forward to now through the duties it sent, with the full model below.
  double estKind=design::estKind;       // 0 = filtered differences (1.11), 1 = model observer, 2 = its tracking-loop fallback
  double estLag=design::estLag;         // lag compensated [s]; nd = round(estLag / Ts) controller ticks
  double estFab=design::estFab;         // observer correction (alpha-beta) bandwidth [Hz]
  double estFtrk=design::estFtrk;       // fallback tracking-loop bandwidth [Hz]
  double estSub=design::estSub;         // RK4 substeps per tick in the observer's model (1 or 2)
  double h=design::pl_h, b1=design::pl_b1, tp=design::pl_tp, eps=design::pl_eps, eps1=design::pl_eps1;
  double stickBand=design::pl_stick_band, stickT=design::pl_stick_T;
  static constexpr int kMaxNd=16;       // longest lag in ticks (16 ms at 1 ms)
  int nd() const {return (int)std::lround(estLag/Ts);}
  double r=0;               // arm reference [rad]
  bool swingUp=true;        // false: balance only (start near upright); losing the pendulum ends the run
  // Supported controller periods and their LQR gains (same Q, R as the design; onboard_rates.h).
  static int periodIndex(double ts){for(int i=0;i<rates::count;i++)if(std::abs(ts-rates::Ts[i])<1e-9)return i;return -1;}
  // Switch the controller period and load the design's LQR gain for it. False if unsupported.
  bool setPeriod(double ts){const int i=periodIndex(ts);if(i<0)return false;Ts=rates::Ts[i];for(int k=0;k<4;k++)K[k]=rates::K[i][k];return true;}
  bool valid(std::string& why) const {
    const double all[]={Ts,velFc,K[0],K[1],K[2],K[3],fricGain,fricBand,fricDutyPos,fricDutyNeg,catchTh1,catchW1,exitTh1,
      safeTh0,E0,kE,aPush,wc,zc,aKick,kickE,wSign,wFric,iMax,uMax,Jp,G,Ja,Beq,tcp,tcn,Rm,Kb,Vs,r,tsp,tsn,stickGain,wStick,aStick,fricStick,
      estKind,estLag,estFab,estFtrk,estSub,h,b1,tp,eps,eps1,stickBand,stickT};
    for(double x:all) if(!std::isfinite(x)){why="Nonfinite onboard parameter";return false;}
    if(periodIndex(Ts)<0){why="Onboard controller period must be 1, 2 or 4 ms (onboard_rates.h)";return false;}
    if(!(estKind==0||estKind==1||estKind==2)){why="estKind must be 0, 1 or 2";return false;}
    if(estKind>0&&(nd()<1||nd()>kMaxNd)){why="estLag must be 1 to 16 controller ticks";return false;}
    if(!(estSub==1||estSub==2)){why="estSub must be 1 or 2";return false;}
    // the observer step takes about 0.5 ms on the ESP32 (measured 2026-11-20): only the 4 ms period leaves room in the
    // 1 ms firmware loop it runs in
    if(estKind>0&&Ts<0.004-1e-9){why="The observer (estKind 1/2) needs the 4 ms controller period; use estKind 0 at 1/2 ms";return false;}
    if(estFab<=0||estFtrk<=0||h<=0||b1<0||tp<0||eps<=0||eps1<=0||stickBand<0||stickT<=0){why="Observer parameter out of range";return false;}
    if(velFc<=0||fricBand<=0||wSign<=0||wFric<=0||Jp<=0||G<=0||Ja<=0||Rm<=0||Vs<=0||iMax<=0||uMax<=0||uMax>1||
       catchTh1<=0||exitTh1<catchTh1||safeTh0<=0||tsp<0||tsn<0||stickGain<0||wStick<=0||aStick<=0||fricStick<0||fricGain<0){why="Onboard parameter out of range";return false;}
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
  // 1.12 observer state (est_observer.slx): duty history (oldest first), arm readings nd ticks back, estimate at
  // the pendulum sample time, start-up flags, fallback tracking loop
  int nd=3;double uh[OnboardParams::kMaxNd+1]{},th0b[OnboardParams::kMaxNd]{},xd[4]{0,3.14159265358979323846,0,0};
  bool obsStarted=false;long obsN=0;double al=0,be=0,alT=0,beT=0;
  bool trkStarted=false;double trkTh[2]{0,0},trkW[2]{0,0};
  // reset() before every run, after setting p (the filter pole and the observer gains are computed once here)
  void reset(){started=false;mode=SwingUp;prev[0]=prev[1]=w[0]=w[1]=0;for(double&v:x)v=0;E=uStab=uSwing=duty=0;
    alpha=std::exp(-2*3.14159265358979323846*p.velFc*p.Ts);
    nd=std::max(1,std::min(OnboardParams::kMaxNd,p.nd()));for(double&v:uh)v=0;for(double&v:th0b)v=0;
    xd[0]=0;xd[1]=3.14159265358979323846;xd[2]=xd[3]=0;obsStarted=false;obsN=0;trkStarted=false;
    const double lam=std::exp(-2*3.14159265358979323846*p.estFab*p.Ts);al=1-lam*lam;be=(1-lam)*(1-lam);
    const double lamT=std::exp(-2*3.14159265358979323846*p.estFtrk*p.Ts);alT=1-lamT*lamT;beT=(1-lamT)*(1-lamT);
    loadModel();}
  static double sat(double v,double lim){return std::min(lim,std::max(-lim,v));}
  static double wrapPi(double a){const double pi=3.14159265358979323846;return a-2*pi*std::floor((a+pi)/(2*pi));}

  // matlab/model/furuta_rhs.m (identified model with stick-slip), the observer's model. Computed in ObsT: float by
  // default, because the ESP32-S3 FPU is single precision only (in double, emulated, one observer step took 2.65 ms
  // of the 4 ms period, measured 2026-11-20 with ONBOARD_BENCH). -DONBOARD_OBSERVER_DOUBLE: double, as Simulink.
#ifdef ONBOARD_OBSERVER_DOUBLE
  using ObsT=double;
#else
  using ObsT=float;
#endif
  struct Model {ObsT Vs,Kb,Rm,tcp,tcn,eps,Beq,Jp,h,b1,tp,eps1,G,Ja,tsp,tsn,stickBand,stickT;};
  Model mp{};
  void loadModel(){mp={(ObsT)p.Vs,(ObsT)p.Kb,(ObsT)p.Rm,(ObsT)p.tcp,(ObsT)p.tcn,(ObsT)p.eps,(ObsT)p.Beq,(ObsT)p.Jp,(ObsT)p.h,
    (ObsT)p.b1,(ObsT)p.tp,(ObsT)p.eps1,(ObsT)p.G,(ObsT)p.Ja,(ObsT)p.tsp,(ObsT)p.tsn,(ObsT)p.stickBand,(ObsT)p.stickT};}
  void rhs(const ObsT s0[4],ObsT d,ObsT dx[4]) const {
    const Model&m=mp;
    const ObsT s=std::sin(s0[1]),c=std::cos(s0[1]),w0=s0[2],w1=s0[3];
    const ObsT i=d!=0?(d*m.Vs-m.Kb*w0)/m.Rm:ObsT(0);
    const ObsT tc=w0>0?m.tcp:m.tcn;
    ObsT fr=tc*std::tanh(w0/m.eps);
    ObsT f0=i-m.Beq*w0-2*m.Jp*s*c*w0*w1+m.h*s*w1*w1;
    const ObsT f1=m.Jp*s*c*w0*w0-m.b1*w1-m.tp*std::tanh(w1/m.eps1)+m.G*s;
    const ObsT m11=m.Ja+m.Jp*s*s,m12=m.h*c,m22=m.Jp,det=m11*m22-m12*m12;
    if(m.tsp>0&&std::abs(w0)<m.stickBand){
      const ObsT need=f0-m12*f1/m.Jp+det*w0/(m.Jp*m.stickT);   // friction that holds the arm
      if(need<=m.tsp&&need>=-m.tsn)fr=need;
    }
    f0-=fr;
    dx[0]=w0;dx[1]=w1;dx[2]=(m22*f0-m12*f1)/det;dx[3]=(-m12*f0+m11*f1)/det;
  }
  void rk4(ObsT s[4],ObsT d,ObsT hh) const {
    ObsT k1[4],k2[4],k3[4],k4[4],t[4];
    rhs(s,d,k1);for(int j=0;j<4;j++)t[j]=s[j]+hh/2*k1[j];
    rhs(t,d,k2);for(int j=0;j<4;j++)t[j]=s[j]+hh/2*k2[j];
    rhs(t,d,k3);for(int j=0;j<4;j++)t[j]=s[j]+hh*k3[j];
    rhs(t,d,k4);for(int j=0;j<4;j++)s[j]=s[j]+hh/6*(k1[j]+2*k2[j]+2*k3[j]+k4[j]);
  }
  // est_observer.slx, tick by tick as est_observer_ref.m. `duty` is still the previous tick's command here.
  void observe(double th0,double th1w) {
    const double Ts=p.Ts;
    const double uNow=sat(duty,.5);                              // duty unit delay + sat
    for(int j=0;j<nd;j++)uh[j]=uh[j+1];uh[nd]=uNow;              // duty history, oldest first
    const double th0then=th0b[0];                                // theta0 nd ticks ago (0 until it holds readings)
    for(int j=0;j<nd-1;j++)th0b[j]=th0b[j+1];th0b[nd-1]=th0;
    double xp[4];
    if(!obsStarted){obsStarted=true;xp[0]=th0;xp[1]=th1w;xp[2]=xp[3]=0;}   // predict: start from the first reading
    else{ObsT q4[4]={(ObsT)xd[0],(ObsT)wrapPi(xd[1]),(ObsT)xd[2],(ObsT)xd[3]};
      const int sub=(int)p.estSub;for(int q=0;q<sub;q++)rk4(q4,(ObsT)uh[0],(ObsT)(Ts/sub));for(int j=0;j<4;j++)xp[j]=q4[j];}
    obsN++;
    double r0=th0then-xp[0];const double r1=wrapPi(th1w-xp[1]);      // compare
    if(obsN<=nd)r0=0;
    xd[0]=xp[0]+al*r0;xd[1]=xp[1]+al*r1;xd[2]=xp[2]+be/Ts*r0;xd[3]=xp[3]+be/Ts*r1;   // correct (L = alpha-beta)
    ObsT xr[4]={(ObsT)xd[0],(ObsT)xd[1],(ObsT)xd[2],(ObsT)xd[3]};  // roll forward to now
    {const int sub=(int)p.estSub;for(int j=1;j<=nd;j++)for(int q=0;q<sub;q++)rk4(xr,(ObsT)uh[j],(ObsT)(Ts/sub));}
    double xm[4]={xr[0],xr[1],xr[2],xr[3]};
    xm[1]=wrapPi(xm[1]);                                         // wrap + NaN guard
    if(!(std::isfinite(xm[0])&&std::isfinite(xm[1])&&std::isfinite(xm[2])&&std::isfinite(xm[3]))){xm[0]=th0;xm[1]=th1w;xm[2]=xm[3]=0;}
    // level 3 fallback: alpha-beta tracking loop, theta1 pushed forward by the lag (always updated, as in Simulink)
    if(!trkStarted){trkStarted=true;trkTh[0]=th0;trkTh[1]=th1w;trkW[0]=trkW[1]=0;}
    const double tp0=trkTh[0]+Ts*trkW[0],tp1=trkTh[1]+Ts*trkW[1];
    const double e0=th0-tp0,e1=wrapPi(th1w-tp1);
    trkTh[0]=tp0+alT*e0;trkTh[1]=wrapPi(tp1+alT*e1);trkW[0]+=beT/Ts*e0;trkW[1]+=beT/Ts*e1;
    if(p.estKind==2){x[0]=trkTh[0];x[1]=wrapPi(trkTh[1]+trkW[1]*nd*Ts);x[2]=trkW[0];x[3]=trkW[1];}
    else{for(int j=0;j<4;j++)x[j]=xm[j];}
  }

  // estimator block: wrap theta1, filtered differences (ctrl_hybrid.slx / estimator); 1.12: or the observer
  void estimate(double th0,double th1) {
    if(p.estKind>0){observe(th0,wrapPi(th1));started=true;return;}
    double th[2]={th0,wrapPi(th1)};
    if(!started){prev[0]=th[0];prev[1]=th[1];w[0]=w[1]=0;started=true;}
    const double a=alpha;
    double d0=th[0]-prev[0],d1=wrapPi(th[1]-prev[1]);
    w[0]=a*w[0]+(1-a)*d0/p.Ts;w[1]=a*w[1]+(1-a)*d1/p.Ts;
    prev[0]=th[0];prev[1]=th[1];
    x[0]=th[0];x[1]=th[1];x[2]=w[0];x[3]=w[1];
  }
  // LQR block + friction compensation block
  // 1.9: the compensation blends from the breakaway duty (arm at rest: Rm tsp / Vs = 13 %, 14 %) to the sliding
  // Coulomb duty (7.4 %, 7.3 %) as the arm speeds up, in the direction of the LQR request. On the rig (firmware 1.8,
  // no compensation) the LQR's small corrections were below the breakaway: the arm stuck for 0.4 s, the pendulum
  // leaned, the arm broke free with a lurch and the LQR saturated. fricStick 0 = the Coulomb-only compensation.
  double lqr() const {
    double u0=-(p.K[0]*(x[0]-p.r)+p.K[1]*x[1]+p.K[2]*x[2]+p.K[3]*x[3]);
    double s=std::tanh(u0/p.fricBand);
    const double coul=p.fricDutyPos*std::max(s,0.)+p.fricDutyNeg*std::min(s,0.);
    const double stat=(p.Rm*p.tsp*std::max(s,0.)+p.Rm*p.tsn*std::min(s,0.))/p.Vs;
    const double g=std::exp(-(x[2]/p.wStick)*(x[2]/p.wStick));
    return u0+p.fricGain*(coul+p.fricStick*g*(stat-coul));
  }
  // swingup_energy.slx / energy law (report section 10)
  double energyLaw() {
    const double th0=x[0],th1=x[1],w0=x[2],w1=x[3];
    E=.5*p.Jp*w1*w1+p.G*(std::cos(th1)-1);
    const double e=E/(2*p.G);
    double a0=-p.aPush*std::min(1.,std::max(-1.,p.kE*(p.E0-e)))*std::tanh(w1*std::cos(th1)/p.wSign);
    if(e< -1+p.kickE)a0=p.aKick;
    a0=a0-p.wc*p.wc*(th0-p.r)-2*p.zc*p.wc*w0;
    // Motor inverse: inertia + viscous + dry friction. The dry friction blends from the breakaway (arm at rest,
    // in the direction of the wanted acceleration) to the sliding Coulomb friction as the arm speeds up (1.8).
    // Without it the kick from rest asked for 11 % duty, below the 13-14 % breakaway, and the arm never moved.
    const double sf=std::tanh(w0/p.wFric);
    const double coul=p.tcp*std::max(sf,0.)+p.tcn*std::min(sf,0.);
    const double sa=std::tanh(a0/p.aStick),stat=p.tsp*std::max(sa,0.)+p.tsn*std::min(sa,0.);
    const double g=std::exp(-(w0/p.wStick)*(w0/p.wStick));
    double i=p.Ja*a0+p.Beq*w0+coul+p.stickGain*g*(stat-coul);
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
    // 1.12: the supervisor needs only the estimate, so decide the mode first and compute only the law it uses
    // (Simulink computes both; the duty is the same). Saves the unused law's emulated-double time each period.
    supervise(first);
    if(mode==Stabilize){uStab=lqr();duty=uStab;}
    else if(mode==SwingUp){uSwing=energyLaw();duty=uSwing;}
    else duty=0.;
    return duty;
  }
};
}
