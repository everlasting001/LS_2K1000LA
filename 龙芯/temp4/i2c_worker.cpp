//============================================================================
// i2c_worker.cpp — I2C 工具类 + 位置插值 + 传感器模拟
//============================================================================
#include "i2c_worker.h"
#include "sensor_sim.h"
#include "i2c_driver.h"
#include "kinematics.h"
#include <QDateTime>
#include <cmath>
#include <cstdio>
#include <cstring>

I2cWorker::I2cWorker() : m_fd(-1), m_sv1_cmd(127), m_sv2_cmd(80) {
    memset(&m_m1, 0, sizeof(m_m1)); memset(&m_m2, 0, sizeof(m_m2));
    memset(&m_m3, 0, sizeof(m_m3)); memset(&m_m4, 0, sizeof(m_m4));
    sim_init();
}

I2cWorker::~I2cWorker() { closeI2C(); }

bool I2cWorker::openI2C() {
    m_fd = i2c_open(FPGA_I2C_BUS, FPGA_I2C_ADDR);
    return m_fd >= 0;
}

void I2cWorker::closeI2C() {
    if (m_fd >= 0) { i2c_close(m_fd); m_fd = -1; }
}

//============================================================================
// 位置插值核心
//============================================================================

// pulsePerUnit: 每物理单位需要多少脉冲 (度→脉冲: arm_deg_to_pulse(1°); mm→脉冲: lift_mm_to_pulse(1mm))
void I2cWorker::startMove(MotorTracker &t, float cur, float delta, quint16 rpm, float pulsePerUnit, float dampMs) {
    t.start_val = cur;
    t.target_val = cur + delta;
    t.start_ms  = QDateTime::currentMSecsSinceEpoch();
    t.was_moving = true;
    t.damp_ms   = dampMs;
    float pulses = fabsf(delta) * pulsePerUnit;
    t.total_ms  = pulses / ((float)rpm / 60.0f * (float)PULSE_PER_REV) * 1000.0f;
    if (t.total_ms < 50.0f) t.total_ms = 50.0f;
    t.cur_val   = cur;
    if (dampMs > 0) {
        t.overshoot = fabsf(delta) * 0.15f + 0.1f;  // 15%超调
        t.damp_sign = (delta >= 0) ? 1.0f : -1.0f;
    }
    t.damp_start_ms = 0;
}

float I2cWorker::interpolate(MotorTracker &t) {
    if (t.total_ms <= 0) {
        if (t.damp_start_ms == 0 || t.damp_ms <= 0) {
            t.was_moving = false;  // 无阻尼直接标记完成
            return t.target_val;
        }
        qint64 damp_elapsed = QDateTime::currentMSecsSinceEpoch() - t.damp_start_ms;
        if (damp_elapsed >= (qint64)t.damp_ms) {
            t.damp_start_ms = 0;
            t.cur_val = t.target_val;
            t.was_moving = false;  // 阻尼完成
            return t.target_val;
        }
        // 衰减震荡: overshoot * e^(-t/tau) * cos(2*pi*t/period)
        float t_norm = (float)damp_elapsed / t.damp_ms;  // 0→1
        float decay  = expf(-t_norm * 4.0f);                 // 指数衰减
        float osc    = cosf(t_norm * 3.14159f * 6.0f);       // 3个周期
        t.cur_val = t.target_val + t.damp_sign * t.overshoot * decay * osc;
        return t.cur_val;
    }

    // 线性插值阶段
    qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - t.start_ms;
    if (elapsed >= (qint64)t.total_ms) {
        // 进入阻尼震荡阶段
        t.damp_start_ms = QDateTime::currentMSecsSinceEpoch();
        t.cur_val = t.target_val + t.damp_sign * t.overshoot;  // 过冲峰值
        t.total_ms = 0;
        return t.cur_val;
    }
    float progress = (float)elapsed / t.total_ms;
    // 轻微S曲线: ease-in-out
    float eased = progress < 0.15f ? (progress*progress/0.15f*0.5f)
                : progress > 0.85f ? (1.0f - (1.0f-progress)*(1.0f-progress)/0.15f*0.5f)
                : progress - 0.075f;
    t.cur_val = t.start_val + (t.target_val - t.start_val) * eased;
    return t.cur_val;
}

//============================================================================
// 轮询
//============================================================================
RobotState I2cWorker::poll() {
    RobotState s; memset(&s, 0, sizeof(s));

    // I2C: 只读 STATUS (在线检测)
    if (m_fd >= 0) {
        uint8_t st;
        if (i2c_read_reg(m_fd, REG_STATUS, &st)) {
            s.connected = true;
            s.status = st;
            s.busy = st & 1;
            s.m2_ok = st & 0x02; s.m3_ok = st & 0x04;
            s.m1_ok = st & 0x20; s.m4_ok = st & 0x40;
        }
    }

    // 电机位置: 本地插值 + 检测完成→归静息
    s.m1_deg = interpolate(m_m1);
    s.m2_mm  = interpolate(m_m2);
    s.m3_deg = interpolate(m_m3);
    s.m4_mm  = interpolate(m_m4);

    // 检查是否已完成运动 → 归静息
    auto checkDone = [](MotorTracker &t, int idx){
        if (t.total_ms <= 0 && t.damp_start_ms == 0) {
            sim_set_moving(idx, false);
        }
    };
    checkDone(m_m1, 0); checkDone(m_m2, 1);
    checkDone(m_m3, 2); checkDone(m_m4, 3);

    // 舵机: 直接命令值
    s.sv1_cmd = m_sv1_cmd;
    s.sv2_cmd = m_sv2_cmd;

    // 传感器模拟
    sim_tick();
    s.voltage_mv = sim_voltage_mv();
    s.cur_m1_ma  = sim_current_ma(0);
    s.cur_m2_ma  = sim_current_ma(1);
    s.cur_m3_ma  = sim_current_ma(2);
    s.cur_m4_ma  = sim_current_ma(3);
    s.temp_m1_c  = sim_temperature_c(0);
    s.temp_m2_c  = sim_temperature_c(1);
    s.temp_m3_c  = sim_temperature_c(2);
    s.temp_m4_c  = sim_temperature_c(3);

    return s;
}

//============================================================================
// 命令 — 记录目标用于插值
//============================================================================

void I2cWorker::cmdArm(qint16 m2_pulse, qint16 m3_pulse, quint16 spd) {
    if (m_fd < 0) return;
    arm_set_target(m_fd, m2_pulse, m3_pulse, spd);
    arm_trigger(m_fd);

    // M2: 脉冲→mm, 无阻尼
    if (m2_pulse != 0) {
        float delta_mm = (float)m2_pulse / (float)PULSE_PER_REV * M2_PITCH_MM;
        startMove(m_m2, m_m2.cur_val, delta_mm, spd, lift_mm_to_pulse(1.0f), 0);
        sim_set_moving(1, true);
    }
    // M3: 脉冲→度, 300ms阻尼
    if (m3_pulse != 0) {
        float delta_deg = (float)m3_pulse / (float)PULSE_PER_REV * 360.0f / M3_RATIO;
        startMove(m_m3, m_m3.cur_val, delta_deg, spd, arm_deg_to_pulse(1.0f), 300);
        sim_set_moving(2, true);
    }
}

void I2cWorker::cmdBase(qint16 m1_pulse, qint16 m4_pulse, quint16 spd) {
    if (m_fd < 0) return;
    base_set_target(m_fd, m1_pulse, m4_pulse, spd);
    base_trigger(m_fd);

    if (m1_pulse != 0) {
        float delta_deg = (float)m1_pulse / (float)PULSE_PER_REV * 360.0f / M1_RATIO;
        startMove(m_m1, m_m1.cur_val, delta_deg, spd, arm_deg_to_pulse(1.0f), 500);  // 500ms阻尼
        sim_set_moving(0, true);
    }
    if (m4_pulse != 0) {
        float delta_mm = (float)m4_pulse / (float)PULSE_PER_REV * (M_PI * M4_ROLLER_DIA_MM);
        startMove(m_m4, m_m4.cur_val, delta_mm, spd, conveyor_mm_to_pulse(1.0f), 0);   // 无阻尼
        sim_set_moving(3, true);
    }
}

void I2cWorker::cmdBoth(qint16 m2, qint16 m3, qint16 m1, qint16 m4, quint16 spd) {
    cmdArm(m2, m3, spd);
    cmdBase(m1, m4, spd);
}

void I2cWorker::cmdServo(quint8 sv1, quint8 sv2) {
    if (m_fd < 0) return;
    // 只更新命令值 (255=不更新)
    if (sv1 != 255) m_sv1_cmd = sv1;
    if (sv2 != 255) m_sv2_cmd = sv2;
    servo_set(m_fd, sv1, sv2);
    servo_trigger(m_fd);
}

void I2cWorker::cmdEstop() {
    if (m_fd < 0) return;
    estop_trigger(m_fd);
    // 急停: 所有电机停止在当前插值位置
    m_m1.target_val = m_m1.cur_val; m_m1.total_ms = 0;
    m_m2.target_val = m_m2.cur_val; m_m2.total_ms = 0;
    m_m3.target_val = m_m3.cur_val; m_m3.total_ms = 0;
    m_m4.target_val = m_m4.cur_val; m_m4.total_ms = 0;
    for (int i=0; i<4; i++) sim_set_moving(i, false);
}

void I2cWorker::cmdEm(int n) {
    if (m_fd < 0 || n < 1 || n > 4) return;
    uint8_t mask = 1 << (n-1);
    printf("[EM] EM%d ON  (reg 0x0C <- 0x%02X)\n", n, mask);
    i2c_write_reg(m_fd, REG_EM, mask);
}

void I2cWorker::cmdEmOff() {
    if (m_fd < 0) return;
    printf("[EM] All OFF (reg 0x0C <- 0x00)\n");
    i2c_write_reg(m_fd, REG_EM, 0);
}

void I2cWorker::cmdBusy(bool on) {
    if (m_fd < 0) return;
    i2c_write_reg(m_fd, 0x0F, on ? 1 : 0);
}
