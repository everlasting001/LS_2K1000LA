// ============================================================
// emm_motor.h — Emm 42 步进电机 MODBUS-RTU 驱动
// ============================================================
// 功能: 通过 UART 以 MODBUS-RTU 协议控制 X42S 闭环步进电机
// 波特率: 115200, 8N1
// CRC:   CRC-16/MODBUS (多项式 0xA001)
// 参考: ModulesBrochure/电机与电机驱动/42步进电机/张大头42闭环/
// ============================================================

#pragma once
#include <Arduino.h>

// -------- CRC-16/MODBUS 校验 --------

// 计算 MODBUS CRC-16 (多项式 0xA001，初始值 0xFFFF)
inline uint16_t modbus_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

// 将 CRC-16 追加到数据末尾 (小端序)
inline void append_modbus_crc(uint8_t *data, size_t len) {
    uint16_t c = modbus_crc16(data, len);
    data[len]     = c;            // 低字节在前
    data[len + 1] = c >> 8;       // 高字节在后
}

// -------- MODBUS 帧收发 --------

// 发送 MODBUS 请求帧，接收并验证应答
// 返回:  接收到的有效字节数 (包含 CRC); 0 = 通信失败
// timeout: 总超时时间 (ms)
// 内部逻辑: 清空接收缓冲 → 发送 → 等待应答 → 校验 CRC → 返回有效长度
inline size_t emm_exchange(
        HardwareSerial &port,
        const uint8_t *tx, size_t tx_len,
        uint8_t *rx, size_t cap,
        uint32_t timeout = 25)
{
    // 清空接收缓冲区
    while (port.available()) port.read();

    // 发送请求帧
    port.write(tx, tx_len);
    port.flush();

    // 接收应答
    size_t n = 0;
    uint32_t start = millis();
    uint32_t last  = start;
    while (millis() - start < timeout) {
        while (port.available() && n < cap) {
            rx[n++] = (uint8_t)port.read();
            last = millis();        // 记录最后一次收到字节的时间
        }
        // 已收到数据且字节间空闲超 3ms，认为应答结束
        if (n && millis() - last >= 3) break;
        delay(1);
    }

    // 最小应答长度: 地址 + 功能码 + CRC = 4 字节
    if (n < 4) return 0;

    // CRC 校验
    uint16_t got = (uint16_t)rx[n - 2] | ((uint16_t)rx[n - 1] << 8);
    return modbus_crc16(rx, n - 2) == got ? n : 0;
}

// -------- 电机控制命令 --------

inline bool emm_enable(HardwareSerial &port, uint8_t addr, bool enable) {
    uint8_t f[13] = {addr,0x10,0x00,0xF3,0x00,0x02,0x04,
                     0xAB,(uint8_t)(enable?1:0),0x00,0x00,0,0};
    append_modbus_crc(f,11);
    uint8_t r[16];
    return emm_exchange(port,f,sizeof(f),r,sizeof(r),100)>=8;
}

// 位置模式: 以指定速度运动到目标脉冲数
//   addr:   MODBUS 从机地址 (出厂默认 1)
//   pulses: 目标脉冲数 (正=正向, 负=反向, 3200 脉冲/圈)
//   speed:  速度 (RPM, 0–3000)
//   relative: true=相对当前位置, false=绝对位置
// 返回: true=命令已确认
inline bool emm_position(
        HardwareSerial &port, uint8_t addr,
        int32_t pulses, uint16_t speed, bool relative)
{
    // 构造 MODBUS 功能码 0x10 (写多个寄存器) 帧
    // 帧格式: addr, 0x10, 0x00FD(寄存器高位/低位), 0x0005(寄存器数), 0x0A(字节数),
    //          方向, 加速度, 速度(H/L), 脉冲(4字节), 模式, 同步标志, CRC
    uint8_t f[19] = {
        addr, 0x10, 0x00, 0xFD,   // 地址 + 功能码 + 寄存器地址 0x00FD
        0x00, 0x05, 0x0A           // 寄存器数量 5 + 数据字节数 10
    };

    // 方向: 0=CW(正转), 1=CCW(反转)
    f[7] = pulses < 0;

    // 加速度档位 (0–255，数值越大加减速越快)
    f[8] = 100;

    // 速度 (16 位，高字节在前)
    f[9]  = speed >> 8;
    f[10] = speed;

    // 脉冲数 (32 位有符号，高字节在前)
    uint32_t p = pulses < 0
        ? (uint32_t)(-(int64_t)pulses)
        : (uint32_t)pulses;
    f[11] = p >> 24;
    f[12] = p >> 16;
    f[13] = p >> 8;
    f[14] = p;

    // 运动模式: 0x01=绝对, 0x02=相对当前位置
    f[15] = relative ? 0x02 : 0x01;

    // 同步标志: 0=立即执行
    f[16] = 0;

    append_modbus_crc(f, 17);

    uint8_t r[16];
    return emm_exchange(port, f, sizeof(f), r, sizeof(r)) >= 8;
}

// 立即停止电机 (功能码 0x06 写单个寄存器 0x00FE=98)
inline void emm_stop(HardwareSerial &port, uint8_t addr) {
    uint8_t f[8] = {
        addr, 0x06,               // 地址 + 功能码 0x06 (写单个寄存器)
        0x00, 0xFE,               // 寄存器地址 0x00FE (立即停止)
        0x98, 0x00                // 停止值 0x0098
    };
    append_modbus_crc(f, 6);
    port.write(f, 8);
    port.flush();
}

// -------- 电机状态读取 --------

// 通用 MODBUS 读取 (功能码 0x03)
//   reg:   起始寄存器地址 (16 位)
//   count: 要读取的寄存器数量 (16 位)
//   data:  接收数据缓冲区
//   capacity: 缓冲区容量
//   length: 输出参数, 实际读取的字节数
// 返回: true=读取成功
inline bool emm_read(
        HardwareSerial &port, uint8_t addr,
        uint16_t reg, uint16_t count,
        uint8_t *data, size_t capacity, size_t &length)
{
    // 构造 MODBUS 功能码 0x03 (读保持寄存器) 帧
    uint8_t f[8] = {
        addr, 0x03,                    // 地址 + 功能码
        (uint8_t)(reg >> 8),           // 起始寄存器地址 (高字节)
        (uint8_t)reg,                  // 起始寄存器地址 (低字节)
        (uint8_t)(count >> 8),         // 寄存器数量 (高字节)
        (uint8_t)count                 // 寄存器数量 (低字节)
    };
    append_modbus_crc(f, 6);

    uint8_t r[32];
    size_t n = emm_exchange(port, f, 8, r, sizeof(r));

    // 验证应答: 最小长度、地址匹配、功能码匹配、数据长度有效
    if (n < 5
        || r[0] != addr
        || r[1] != 0x03
        || r[2] > capacity
        || n < (size_t)r[2] + 5) {
        return false;
    }

    length = r[2];
    memcpy(data, r + 3, length);
    return true;
}

// 读取电机实时位置 (寄存器 0x0036, 3 个寄存器 = 6 字节)
// 返回值: 0–65535 (对应 0°–360°, 精度 = 360/65536 ≈ 0.0055°)
inline bool emm_read_position(
        HardwareSerial &port, uint8_t addr, uint16_t &position)
{
    uint8_t d[8];
    size_t n = 0;
    if (!emm_read(port, addr, 0x0036, 3, d, sizeof(d), n) || n < 6) {
        return false;
    }
    position = ((uint16_t)d[3] << 8) | d[4];
    return true;
}

// 读取电机状态标志 (寄存器 0x003A, 1 个寄存器 = 2 字节)
// 位定义: bit2=位置到达, bit3=堵转（与旧版已验证工程保持一致）
inline bool emm_read_status(
        HardwareSerial &port, uint8_t addr, uint8_t &status)
{
    uint8_t d[2];
    size_t n = 0;
    if (!emm_read(port, addr, 0x003A, 1, d, sizeof(d), n) || n < 2) {
        return false;
    }
    status = d[1];
    return true;
}
