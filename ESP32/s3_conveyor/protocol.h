// ============================================================
// protocol.h — ESP-NOW 无线协议定义 (v3)
// ============================================================
// 功能: 定义 ESP32-S3 三机之间的命令/状态包格式及 CRC 校验
// 协议版本: 3
// CRC:  CRC-8/CCITT (多项式 0x07)
// ============================================================

#pragma once
#include <Arduino.h>

// -------- 协议常量 --------

// 协议版本号 (三机必须一致)
static const uint8_t PROTOCOL_VERSION = 3;

// 命令包魔数 (路由器 → 执行端)
static const uint8_t CMD_MAGIC    = 0xAA;

// 状态包魔数 (执行端 → 路由器)
static const uint8_t STATUS_MAGIC = 0xBB;

// -------- 命令代码 --------

enum CmdCode : uint8_t {
    CMD_MOVE      = 0x01,   // 电机位置移动
    CMD_HOME      = 0x02,   // 回原点
    CMD_ESTOP     = 0x03,   // 急停
    CMD_READ      = 0x04,   // 读取状态/参数
    CMD_SERVO     = 0x05,   // 舵机控制
    CMD_GET_PARAM = 0x06,   // 获取参数
    CMD_SOLENOID  = 0x07,   // 电磁铁控制
    CMD_REBOOT    = 0xFE    // 远程重启 (预留)
};

// -------- 设备 ID --------

enum DeviceId : uint8_t {
    DEV_BASE      = 1,      // 底盘路由器 S3
    DEV_ARM       = 2,      // 上臂执行端 S3
    DEV_CONVEYOR  = 3       // 传送带执行端 S3
};

// -------- 命令标志位 --------

#define FLAG_M2_ENABLE  0x01    // 使能升降电机 M2
#define FLAG_M3_ENABLE  0x02    // 使能小臂电机 M3
#define FLAG_RELATIVE   0x04    // 相对运动模式 (否则绝对)
#define FLAG_HOME_TRIG  0x08    // 触发回原点
#define FLAG_ESTOP      0x10    // 急停标志

// -------- 数据包结构 (单字节对齐, 紧凑传输) --------

#pragma pack(push, 1)

// 命令包: 路由器 → 执行端 (22 字节)
struct CmdPacket {
    uint8_t  magic;              // 魔数 0xAA
    uint8_t  version;            // 协议版本号
    uint8_t  cmd;                // 命令代码 (CmdCode)
    uint8_t  seq;                // 序列号 (用于去重/确认)
    int32_t  value1;             // 电机1 目标位置 (脉冲数)
    int32_t  value2;             // 电机2 目标位置 (脉冲数)
    uint16_t speed1;             // 电机1 速度 (RPM) / 电磁铁动作时长 (10ms)
    uint16_t speed2;             // 电机2 速度 (RPM) / 电磁铁冷却时长 (10ms)
    uint16_t aux1;               // 辅助参数1 (舵机角度1 / 电磁铁掩码)
    uint16_t aux2;               // 辅助参数2 (舵机角度2)
    uint8_t  flags;              // 命令标志位
    uint8_t  crc8;               // CRC-8/CCITT 校验 (覆盖前 21 字节)
};

// 状态包: 执行端 → 路由器 (24 字节)
struct StatusPacket {
    uint8_t  magic;              // 魔数 0xBB
    uint8_t  version;            // 协议版本号
    uint8_t  device;             // 设备 ID (DeviceId)
    uint8_t  seq;                // 对应最后一条命令的序列号
    uint16_t pos1;               // 电机1 实时位置 (编码器值, 0–65535)
    uint16_t pos2;               // 电机2 实时位置
    uint16_t speed1;             // 电机1 实时速度 (RPM)
    uint16_t speed2;             // 电机2 实时速度 (RPM)
    uint16_t current1;           // 电机1 电流 (预留)
    uint16_t current2;           // 电机2 电流 (预留)
    uint8_t  stat1;              // 电机1 状态标志
    uint8_t  stat2;              // 电机2 状态标志
    uint16_t aux1;               // 辅助状态1 (舵机1角度 / 电磁铁掩码)
    uint16_t aux2;               // 辅助状态2 (舵机2角度 / 电磁铁状态)
    uint8_t  error;              // 错误码
    uint8_t  crc8;               // CRC-8/CCITT 校验 (覆盖前 23 字节)
};

#pragma pack(pop)

// 编译期包大小验证 (防止结构体对齐问题)
static_assert(sizeof(CmdPacket)    == 22, "CmdPacket size changed");
static_assert(sizeof(StatusPacket) == 24, "StatusPacket size changed");

// -------- CRC-8/CCITT 校验函数 --------
// 多项式: 0x07 (x^8 + x^2 + x + 1)
// 初始值: 0x00

inline uint8_t crc8_ccitt(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x80)
                ? (uint8_t)((crc << 1) ^ 0x07)
                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// -------- 数据包辅助函数 --------

// 计算并填入 CRC 校验字节 (覆盖除 CRC 字段外的所有字段)
template<typename T>
inline void packet_set_crc(T &p) {
    p.crc8 = crc8_ccitt((uint8_t *)&p, sizeof(T) - 1);
}

// 验证数据包的完整性和合法性
// 检查: 魔数匹配 + 版本匹配 + CRC 校验通过
template<typename T>
inline bool packet_valid(const T &p, uint8_t magic) {
    return p.magic   == magic
        && p.version == PROTOCOL_VERSION
        && p.crc8    == crc8_ccitt((const uint8_t *)&p, sizeof(T) - 1);
}

// 验证 4 位 one-hot 掩码 (最多一位高电平, 不能全零)
inline bool valid_one_hot4(uint8_t mask) {
    mask &= 0x0F;
    return mask && (mask & (mask - 1)) == 0;
}
