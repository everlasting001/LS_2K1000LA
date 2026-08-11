#pragma once
#include <Arduino.h>
static const uint8_t EMM_SUM=0x6B;
inline size_t emm_xchg(HardwareSerial&s,const uint8_t*tx,size_t tn,uint8_t*rx,size_t cap,uint32_t timeout=100){while(s.available())s.read();s.write(tx,tn);s.flush();size_t n=0;uint32_t st=millis(),last=st;while(millis()-st<timeout){while(s.available()&&n<cap){rx[n++]=(uint8_t)s.read();last=millis();}if(n&&millis()-last>=3)break;delay(1);}return n>=3&&rx[n-1]==EMM_SUM?n:0;}
inline bool emm_ack(HardwareSerial&s,uint8_t a,const uint8_t*f,size_t fn,uint8_t code){uint8_t r[8];size_t n=emm_xchg(s,f,fn,r,sizeof(r));return n>=4&&r[0]==a&&r[1]==code&&r[2]==0x02;}
inline bool emm_enable(HardwareSerial&s,uint8_t a,bool en=true){uint8_t f[6]={a,0xF3,0xAB,(uint8_t)(en?1:0),0,EMM_SUM};return emm_ack(s,a,f,sizeof(f),0xF3);}
inline bool emm_position(HardwareSerial&s,uint8_t a,int32_t pulses,uint16_t speed,bool relative){uint32_t p=pulses<0?(uint32_t)(-(int64_t)pulses):(uint32_t)pulses;uint8_t f[13]={a,0xFD,(uint8_t)(pulses<0),(uint8_t)(speed>>8),(uint8_t)speed,20,(uint8_t)(p>>24),(uint8_t)(p>>16),(uint8_t)(p>>8),(uint8_t)p,(uint8_t)(relative?2:1),0,EMM_SUM};return emm_enable(s,a,true)&&emm_ack(s,a,f,sizeof(f),0xFD);}
inline void emm_stop(HardwareSerial&s,uint8_t a){uint8_t f[5]={a,0xFE,0x98,0,EMM_SUM},r[8];emm_xchg(s,f,sizeof(f),r,sizeof(r));}
inline bool emm_read_position(HardwareSerial&s,uint8_t a,uint16_t&pos){uint8_t f[3]={a,0x36,EMM_SUM},r[12];size_t n=emm_xchg(s,f,sizeof(f),r,sizeof(r));if(n<8||r[0]!=a||r[1]!=0x36)return false;uint32_t p=((uint32_t)r[3]<<24)|((uint32_t)r[4]<<16)|((uint32_t)r[5]<<8)|r[6];pos=(uint16_t)p;return true;}
inline bool emm_read_status(HardwareSerial&s,uint8_t a,uint8_t&status){uint8_t f[3]={a,0x3A,EMM_SUM},r[8];size_t n=emm_xchg(s,f,sizeof(f),r,sizeof(r));if(n<4||r[0]!=a||r[1]!=0x3A)return false;status=r[2];return true;}
