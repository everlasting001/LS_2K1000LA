//============================================================================
// i2c_driver.c — FPGA I2C 读写驱动
//
// 编译: gcc -o test_i2c i2c_driver.c test_i2c.c
// 运行: sudo ./test_i2c
//============================================================================
#include "i2c_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

//============================================================================
// I2C 基本操作
//============================================================================

int i2c_open(const char *bus, uint8_t addr) {
    int fd = open(bus, O_RDWR);
    if (fd < 0) {
        perror("open I2C bus");
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        perror("ioctl I2C_SLAVE");
        close(fd);
        return -1;
    }
    printf("[I2C] Opened %s, slave=0x%02X, fd=%d\n", bus, addr, fd);
    return fd;
}

void i2c_close(int fd) {
    if (fd >= 0) close(fd);
}

bool i2c_write_reg(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (write(fd, buf, 2) != 2) {
        fprintf(stderr, "[I2C] Write reg 0x%02X failed\n", reg);
        return false;
    }
    return true;
}

bool i2c_write_reg16(int fd, uint8_t reg_h, uint8_t reg_l, int16_t val) {
    // 大端: 先写高字节到 reg_h, 再写低字节到 reg_l
    //   用两次单字节写, 因为 FPGA i2c_slave 每次只处理一个字节
    uint8_t hi = (val >> 8) & 0xFF;
    uint8_t lo = val & 0xFF;
    if (!i2c_write_reg(fd, reg_h, hi)) return false;
    if (!i2c_write_reg(fd, reg_l, lo)) return false;
    return true;
}

bool i2c_read_reg(int fd, uint8_t reg, uint8_t *val) {
    // 使用 I2C_RDWR 组合消息 (repeated START, 无 STOP 间隙)
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data rdwr;

    msgs[0].addr  = FPGA_I2C_ADDR;
    msgs[0].flags = 0;                  // write
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = FPGA_I2C_ADDR;
    msgs[1].flags = I2C_M_RD;          // read
    msgs[1].len   = 1;
    msgs[1].buf   = val;

    rdwr.msgs  = msgs;
    rdwr.nmsgs = 2;

    if (ioctl(fd, I2C_RDWR, &rdwr) < 0) {
        // 静默失败, 避免刷屏 (每 100ms 可能有一次超时)
        return false;
    }
    return true;
}

bool i2c_read_reg16(int fd, uint8_t reg_h, uint8_t reg_l, int16_t *val) {
    uint8_t hi, lo;
    if (!i2c_read_reg(fd, reg_h, &hi)) return false;
    if (!i2c_read_reg(fd, reg_l, &lo)) return false;
    *val = (int16_t)((hi << 8) | lo);
    return true;
}

//============================================================================
// 运动命令封装
//============================================================================

bool arm_set_target(int fd, int16_t m2_pulse, int16_t m3_pulse, uint16_t speed) {
    printf("[ARM] Set M2=%d M3=%d speed=%u\n", m2_pulse, m3_pulse, speed);
    if (!i2c_write_reg16(fd, REG_M2_PULSE_H, REG_M2_PULSE_L, m2_pulse)) return false;
    if (!i2c_write_reg16(fd, REG_M3_PULSE_H, REG_M3_PULSE_L, m3_pulse)) return false;
    if (!i2c_write_reg16(fd, REG_SPEED_H, REG_SPEED_L, speed)) return false;
    return true;
}

bool base_set_target(int fd, int16_t m1_pulse, int16_t m4_pulse, uint16_t speed) {
    printf("[BASE] Set M1=%d M4=%d speed=%u\n", m1_pulse, m4_pulse, speed);
    if (!i2c_write_reg16(fd, REG_M1_PULSE_H, REG_M1_PULSE_L, m1_pulse)) return false;
    if (!i2c_write_reg16(fd, REG_M4_PULSE_H, REG_M4_PULSE_L, m4_pulse)) return false;
    if (!i2c_write_reg16(fd, REG_SPEED_H, REG_SPEED_L, speed)) return false;
    return true;
}

bool servo_set(int fd, uint8_t sv1_ang, uint8_t sv2_ang) {
    printf("[SERVO] SV1=%u° SV2=%u°\n", sv1_ang, sv2_ang);
    if (!i2c_write_reg(fd, REG_SERVO1, sv1_ang)) return false;
    if (!i2c_write_reg(fd, REG_SERVO2, sv2_ang)) return false;
    return true;
}

bool servo_trigger(int fd) {
    printf("[SERVO] Trigger\n");
    return i2c_write_reg(fd, REG_CMD, CMD_SERVO);
}

bool arm_trigger(int fd) {
    printf("[ARM] Trigger\n");
    return i2c_write_reg(fd, REG_CMD, CMD_ARM);
}

bool base_trigger(int fd) {
    printf("[BASE] Trigger\n");
    return i2c_write_reg(fd, REG_CMD, CMD_BASE);
}

bool both_trigger(int fd) {
    printf("[BOTH] Trigger (ARM+BASE)\n");
    return i2c_write_reg(fd, REG_CMD, CMD_ARM | CMD_BASE);
}

bool estop_trigger(int fd) {
    printf("[ESTOP!]\n");
    return i2c_write_reg(fd, REG_CMD, CMD_ESTOP);
}

bool arm_read_pos(int fd, int16_t *m2_pos, int16_t *m3_pos) {
    if (!i2c_read_reg16(fd, REG_M2_POS_H, REG_M2_POS_L, m2_pos)) return false;
    if (!i2c_read_reg16(fd, REG_M3_POS_H, REG_M3_POS_L, m3_pos)) return false;
    return true;
}

bool base_read_pos(int fd, int16_t *m1_pos, int16_t *m4_pos) {
    if (!i2c_read_reg16(fd, REG_M1_POS_H, REG_M1_POS_L, m1_pos)) return false;
    if (!i2c_read_reg16(fd, REG_M4_POS_H, REG_M4_POS_L, m4_pos)) return false;
    return true;
}
