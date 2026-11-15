#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <vector>

namespace fw {
constexpr size_t MAX_PACKET=2048, HEADER=18;
enum Message : uint8_t { Command=1, Ack=2, Event=3, Samples=4, Burst=5 };
inline uint32_t crc32(const uint8_t* p,size_t n) {
  uint32_t c=0xffffffff;
  while(n--) { c^=*p++; for(int b=0;b<8;b++) c=(c>>1)^((c&1)?0xedb88320:0); }
  return c^0xffffffff;
}
template<class T> inline void put(std::vector<uint8_t>& p,T v) {
  uint8_t a[sizeof(T)]; memcpy(a,&v,sizeof(T)); p.insert(p.end(),a,a+sizeof(T));
}
template<class T> inline T get(const uint8_t* p) { T v; memcpy(&v,p,sizeof(T)); return v; }
inline std::vector<uint8_t> cobs(const std::vector<uint8_t>& in) {
  std::vector<uint8_t> out(1,0); size_t at=0; uint8_t code=1;
  for(auto b:in) {
    if(b==0) { out[at]=code; at=out.size(); out.push_back(0); code=1; }
    else { out.push_back(b); if(++code==255) {out[at]=code; at=out.size();out.push_back(0);code=1;} }
  }
  out[at]=code; out.push_back(0); return out;
}
inline bool uncobs(const uint8_t* p,size_t n,std::vector<uint8_t>& out) {
  out.clear(); size_t i=0;
  while(i<n) {
    unsigned code=p[i++]; if(!code || i+code-1>n) return false;
    for(unsigned j=1;j<code;j++) { if(p[i]==0)return false;out.push_back(p[i++]); }
    if(code<255 && i<n)out.push_back(0);
    if(out.size()>MAX_PACKET)return false;
  }
  return true;
}
struct Packet { uint8_t type=0;uint16_t flags=0;uint64_t session=0;uint32_t sequence=0;std::vector<uint8_t> payload; };
inline std::vector<uint8_t> encode(const Packet& f) {
  if(f.payload.size()>MAX_PACKET-HEADER-4)return {};
  std::vector<uint8_t> p; p.reserve(HEADER+f.payload.size()+4);
  put<uint8_t>(p,1);put<uint8_t>(p,f.type);put<uint16_t>(p,f.flags);
  put<uint64_t>(p,f.session);put<uint32_t>(p,f.sequence);put<uint16_t>(p,(uint16_t)f.payload.size());
  p.insert(p.end(),f.payload.begin(),f.payload.end());put<uint32_t>(p,crc32(p.data(),p.size())); return cobs(p);
}
inline bool decode(const uint8_t* data,size_t n,Packet& f) {
  std::vector<uint8_t> p;
  if(!uncobs(data,n,p)||p.size()<22||p[0]!=1)return false;
  const size_t len=get<uint16_t>(&p[16]);
  if(p.size()!=HEADER+len+4||crc32(p.data(),p.size()-4)!=get<uint32_t>(&p[p.size()-4]))return false;
  f.type=p[1];f.flags=get<uint16_t>(&p[2]);f.session=get<uint64_t>(&p[4]);f.sequence=get<uint32_t>(&p[12]);
  f.payload.assign(p.begin()+HEADER,p.end()-4);return true;
}
}
