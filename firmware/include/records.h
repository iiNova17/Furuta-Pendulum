#pragma once
#include <algorithm>
#include <cmath>
#include "protocol.h"
namespace hw {
struct Snapshot {
  uint64_t t=0,adcTime=0,pendTime=0,applyTime=0;
  uint32_t seq=0,flags=0,limits=0,configId=0,runId=0;int64_t count=0;
  uint16_t pend=0,adc=0,mag=0;uint8_t status=0,agc=0,state=1;
  float mv=NAN,filtered=NAN,requested=0,applied=0;
};
inline void pack(std::vector<uint8_t>& p,const Snapshot&s) {
  fw::put<uint64_t>(p,s.t);fw::put<uint32_t>(p,s.seq);fw::put<int64_t>(p,s.count);
  fw::put<uint16_t>(p,s.pend);fw::put<uint16_t>(p,s.adc);fw::put<float>(p,s.mv);fw::put<float>(p,s.filtered);
  fw::put<float>(p,s.requested);fw::put<float>(p,s.applied);
  fw::put<uint32_t>(p,(uint32_t)std::min<uint64_t>(UINT32_MAX,s.t-s.pendTime));
  fw::put<uint32_t>(p,(uint32_t)std::min<uint64_t>(UINT32_MAX,s.t-s.adcTime));
  fw::put<uint8_t>(p,s.status);fw::put<uint8_t>(p,s.agc);fw::put<uint16_t>(p,s.mag);
  fw::put<uint32_t>(p,s.flags);fw::put<uint32_t>(p,s.limits);
  fw::put<uint64_t>(p,s.applyTime);fw::put<uint64_t>(p,s.adcTime);fw::put<uint64_t>(p,s.pendTime);
  fw::put<uint8_t>(p,s.state);fw::put<uint8_t>(p,0);fw::put<uint16_t>(p,0);
}
struct AdcRecord {uint64_t t;uint16_t raw;float mv,requested,applied;};
inline void pack(std::vector<uint8_t>& p,const AdcRecord&s) {
  fw::put<uint64_t>(p,s.t);fw::put<uint16_t>(p,s.raw);fw::put<float>(p,s.mv);
  fw::put<float>(p,s.requested);fw::put<float>(p,s.applied);
}
}
