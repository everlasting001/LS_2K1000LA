//============================================================================
// i2c_driver.h — FPGA I2C 寄存器定义 & 读写封装
//
// 总线: 龙芯 I2C-1 (LS2K_IIC1, SCL=D20, SDA=C22)
// 从机: FPGA 地址 0x20
//============================================================================
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C 总线参数
#define FPGA_I2C_BUS   "/dev/i2c-1"
#define FPGA_I2C_ADDR  0x20

//============================================================================
// 写寄存器 (龙芯 → FPGA)
//============================================================================
#define REG_CMD         0x00   // [7]=arm [6]=base [3]=servo [2]=EM [1]=estop
#define REG_M2_PULSE_H  0x01   // M2 目标脉冲高字节
#define REG_M2_PULSE_L  0x02
#define REG_M3_PULSE_H  0x03
#define REG_M3_PULSE_L  0x04
#define REG_SPEED_H     0x05   // 速度 RPM 高字节
#define REG_SPEED_L     0x06
// 0x07 保留
#define REG_M1_PULSE_H  0x08
#define REG_M1_PULSE_L  0x09
#define REG_M4_PULSE_H  0x0A
#define REG_M4_PULSE_L  0x0B
#define REG_EM          0x0C   // [3:0]=推杆 (直连引脚, 写0关断)
#define REG_SERVO1      0x0D   // 舵机1 角度 0-270
#define REG_SERVO2      0x0E   // 舵机2 角度 0-180

// CMD 位定义
#define CMD_ARM    0x80   // bit7: 发送 M2+M3 帧
#define CMD_BASE   0x40   // bit6: 发送 M1+M4 帧
#define CMD_SERVO  0x08   // bit3: 发送舵机帧
#define CMD_ESTOP  0x02   // bit1: 急停

//============================================================================
// 读寄存器 (FPGA → 龙芯)
//============================================================================
#define REG_M2_POS_H  0x10
#define REG_M2_POS_L  0x11
#define REG_M3_POS_H  0x12
#define REG_M3_POS_L  0x13
#define REG_M1_POS_H  0x18
#define REG_M1_POS_L  0x19
#define REG_M4_POS_H  0x20
#define REG_M4_POS_L  0x21
#define REG_STATUS    0x28

//============================================================================
// API
//============================================================================

// 打开 I2C, 返回 fd, 失败返回 -1
int i2c_open(const char *bus, uint8_t addr);

// 关闭 I2C
void i2c_close(int fd);

// 写单个寄存器 (8-bit)
bool i2c_write_reg(int fd, uint8_t reg, uint8_t val);

// 写 16-bit 寄存器 (大端, 分两个连续地址)
bool i2c_write_reg16(int fd, uint8_t reg_h, uint8_t reg_l, int16_t val);

// 读单个寄存器
bool i2c_read_reg(int fd, uint8_t reg, uint8_t *val);

// 读 16-bit 寄存器 (大端)
bool i2c_read_reg16(int fd, uint8_t reg_h, uint8_t reg_l, int16_t *val);

//============================================================================
// 运动命令封装
//============================================================================

// 设置目标脉冲 (写寄存器, 不触发)
bool arm_set_target(int fd, int16_t m2_pulse, int16_t m3_pulse, uint16_t speed);
bool base_set_target(int fd, int16_t m1_pulse, int16_t m4_pulse, uint16_t speed);

// 舵机设置
bool servo_set(int fd, uint8_t sv1_ang, uint8_t sv2_ang);

// 触发发送 (写 CMD 寄存器)
bool arm_trigger(int fd);    // CMD bit7
bool base_trigger(int fd);   // CMD bit6
bool both_trigger(int fd);   // CMD bit7|bit6 (ARM+BASE 同时)
bool servo_trigger(int fd);  // CMD bit3
bool estop_trigger(int fd);  // CMD bit1

// 读回位置
bool arm_read_pos(int fd, int16_t *m2_pos, int16_t *m3_pos);
bool base_read_pos(int fd, int16_t *m1_pos, int16_t *m4_pos);

#ifdef __cplusplus
}
#endif
