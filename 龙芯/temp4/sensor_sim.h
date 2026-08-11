//============================================================================
// sensor_sim.h — 传感器模拟器 (电压/电流/温度, 预生成LUT, 温度慢漂)
//============================================================================
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void sim_init(void);
void sim_set_moving(int motor_idx, bool moving);  // 0=M1 1=M2 2=M3 3=M4
void sim_tick(void);                               // 100ms 步进

uint16_t sim_voltage_mv(void);                     // 10500-11500 mV
uint16_t sim_current_ma(int motor_idx);            // 静息5-25, 运动140-220mA
float    sim_temperature_c(int motor_idx);         // 30-40°C, 漂移<1°C/min

#ifdef __cplusplus
}
#endif
