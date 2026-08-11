//============================================================================
// kinematics.h — 脉冲换算 (基于 config.md 机械参数)
//
// 步进电机: 1.8°, 16细分 = 3200 pulse/rev
// M1/M3:    减速比 3.75:1 (60:16)
// M2:       丝杆导程 2mm, 直驱 1:1
// M4:       传送带滚筒 Ø21mm
//============================================================================
#pragma once
#include <stdint.h>
#include <math.h>

#define PULSE_PER_REV     3200
#define M_PI_F            3.14159265f

// 减速比
#define M1_RATIO          3.75f
#define M2_PITCH_MM       2.0f
#define M3_RATIO          3.75f
#define M4_ROLLER_DIA_MM  21.0f

//============================================================================
// 物理量 → 脉冲 (发送命令用)
//============================================================================

// 大臂/小臂: 角度(°) → 脉冲
static inline int32_t arm_deg_to_pulse(float deg) {
    return (int32_t)(deg * M1_RATIO / 360.0f * PULSE_PER_REV);
}

// 升降: 高度(mm) → 脉冲
static inline int32_t lift_mm_to_pulse(float mm) {
    return (int32_t)(mm / M2_PITCH_MM * PULSE_PER_REV);
}

// 传送带: 距离(mm) → 脉冲
static inline int32_t conveyor_mm_to_pulse(float mm) {
    return (int32_t)(mm / (M_PI_F * M4_ROLLER_DIA_MM) * PULSE_PER_REV);
}

//============================================================================
// 编码器 → 物理量 (读取位置用)
//============================================================================

// 编码器值 (0-65535) → 电机轴角度(°)
static inline float encoder_to_motor_deg(uint16_t enc) {
    return enc * 360.0f / 65536.0f;
}

// 编码器值 → 关节角度(°) (M1/M3 带减速比)
static inline float encoder_to_joint_deg(uint16_t enc) {
    return encoder_to_motor_deg(enc) / M1_RATIO;
}

// 编码器值 → 升降高度(mm)
static inline float encoder_to_lift_mm(uint16_t enc) {
    return encoder_to_motor_deg(enc) / 360.0f * M2_PITCH_MM;
}

// 编码器值 → 传送带位移(mm)
static inline float encoder_to_conveyor_mm(uint16_t enc) {
    return encoder_to_motor_deg(enc) / 360.0f * (M_PI_F * M4_ROLLER_DIA_MM);
}

//============================================================================
// 编码器差值 → 相对脉冲 (用于姿态间移动)
//============================================================================

// 已知当前编码器和目标编码器, 计算相对脉冲
static inline int16_t encoder_delta_to_pulse(uint16_t cur_enc, uint16_t tgt_enc) {
    int32_t diff = (int32_t)tgt_enc - (int32_t)cur_enc;
    // 走最短路径
    if (diff > 32768)  diff -= 65536;
    if (diff < -32768) diff += 65536;
    return (int16_t)(diff * PULSE_PER_REV / 65536);
}
