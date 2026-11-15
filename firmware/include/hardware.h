#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <driver/adc.h>
#include <driver/pcnt.h>
#include <esp_adc_cal.h>
#include <esp_timer.h>
#include <soc/pcnt_struct.h>
#include "protocol.h"
#include "records.h"
namespace hw {
constexpr int ADC_PIN=4,A=5,B=6,SDA_PIN=8,SCL_PIN=9,PWM=15,IN1=16,IN2=17,STBY=18;
constexpr int LIMIT=30000;
inline portMUX_TYPE counterMux=portMUX_INITIALIZER_UNLOCKED;
inline int64_t countBase=0;
inline bool encoderOk=false;
inline esp_adc_cal_characteristics_t adcCalibration{};
inline esp_adc_cal_value_t calibrationSource;
inline void serviceCounter() {
  if(PCNT.int_st.val&1) {
    uint32_t events=0;pcnt_get_event_status(PCNT_UNIT_0,&events);
    if(events&PCNT_EVT_H_LIM)countBase+=LIMIT;
    if(events&PCNT_EVT_L_LIM)countBase-=LIMIT;
    PCNT.int_clr.val=1;
  }
}
inline void counterISR(void*) {
  portENTER_CRITICAL_ISR(&counterMux);serviceCounter();portEXIT_CRITICAL_ISR(&counterMux);
}
inline int64_t count() {
  portENTER_CRITICAL(&counterMux);
  serviceCounter();int16_t v=0;pcnt_get_counter_value(PCNT_UNIT_0,&v);
  if(PCNT.int_st.val&1){serviceCounter();pcnt_get_counter_value(PCNT_UNIT_0,&v);}
  int64_t total=countBase+v;
  portEXIT_CRITICAL(&counterMux);return total;
}
// Make the current position read as count 0 (used when the arm is centered).
inline void zeroCount() {
  portENTER_CRITICAL(&counterMux);
  serviceCounter();int16_t v=0;pcnt_get_counter_value(PCNT_UNIT_0,&v);
  if(PCNT.int_st.val&1){serviceCounter();pcnt_get_counter_value(PCNT_UNIT_0,&v);}
  countBase=-(int64_t)v;
  portEXIT_CRITICAL(&counterMux);
}
inline bool beginCounter() {
  pinMode(A,INPUT_PULLUP);pinMode(B,INPUT_PULLUP);
  pcnt_config_t p{};p.unit=PCNT_UNIT_0;p.counter_h_lim=LIMIT;p.counter_l_lim=-LIMIT;
  p.lctrl_mode=PCNT_MODE_KEEP;p.hctrl_mode=PCNT_MODE_REVERSE;
  p.channel=PCNT_CHANNEL_0;p.pulse_gpio_num=A;p.ctrl_gpio_num=B;p.pos_mode=PCNT_COUNT_INC;p.neg_mode=PCNT_COUNT_DEC;
  if(pcnt_unit_config(&p)!=ESP_OK)return false;
  p.channel=PCNT_CHANNEL_1;p.pulse_gpio_num=B;p.ctrl_gpio_num=A;p.pos_mode=PCNT_COUNT_DEC;p.neg_mode=PCNT_COUNT_INC;
  if(pcnt_unit_config(&p)!=ESP_OK)return false;
  pcnt_counter_pause(PCNT_UNIT_0);pcnt_counter_clear(PCNT_UNIT_0);pcnt_set_filter_value(PCNT_UNIT_0,100);
  pcnt_filter_enable(PCNT_UNIT_0);pcnt_event_enable(PCNT_UNIT_0,PCNT_EVT_H_LIM);pcnt_event_enable(PCNT_UNIT_0,PCNT_EVT_L_LIM);
  pcnt_isr_handle_t handle=nullptr;
  if(pcnt_isr_register(counterISR,nullptr,0,&handle)!=ESP_OK)return false;
  pcnt_intr_enable(PCNT_UNIT_0);pcnt_counter_resume(PCNT_UNIT_0);return true;
}
inline void output(double duty) {
  if(!std::isfinite(duty)||std::abs(duty)<1e-8) {
    digitalWrite(STBY,LOW);ledcWrite(0,0);digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);return;
  }
  digitalWrite(IN1,duty>0);digitalWrite(IN2,duty<0);
  ledcWrite(0,(uint32_t)std::lround(std::abs(duty)*1023));digitalWrite(STBY,HIGH);
}
inline bool readReg(uint8_t reg,uint8_t* out,size_t n) {
  Wire.beginTransmission(0x36);Wire.write(reg);
  if(Wire.endTransmission(false))return false;
  if(Wire.requestFrom((uint8_t)0x36,n,true)!=n){while(Wire.available())Wire.read();return false;}
  for(size_t i=0;i<n;i++)out[i]=Wire.read();return true;
}
inline void begin() {
  digitalWrite(STBY,LOW);pinMode(STBY,OUTPUT);
  pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);pinMode(PWM,OUTPUT);
  ledcSetup(0,20000,10);ledcAttachPin(PWM,0);output(0);
  encoderOk=beginCounter();
  adc1_config_width(ADC_WIDTH_BIT_12);adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_12);
  calibrationSource=esp_adc_cal_characterize(ADC_UNIT_1,ADC_ATTEN_DB_12,ADC_WIDTH_BIT_12,1100,&adcCalibration);
  Wire.setTimeOut(3);Wire.begin(SDA_PIN,SCL_PIN,400000);
}
}
