//============================================================================
// i2c_worker.h — I2C 工具类 + 位置插值 + 传感器模拟
//============================================================================
#pragma once
#include <QtGlobal>
#include <QElapsedTimer>

struct RobotState {
    bool connected;
    quint8 status;
    bool m1_ok, m2_ok, m3_ok, m4_ok, busy;

    // 电机位置 (本地插值计算, 非I2C回读)
    float m1_deg, m2_mm, m3_deg, m4_mm;
    float m1_spd, m2_spd, m3_spd, m4_spd;  // 当前RPM

    // 舵机 (命令值直接显示)
    quint8 sv1_cmd, sv2_cmd;

    // 传感器模拟
    quint16 voltage_mv;
    quint16 cur_m1_ma, cur_m2_ma, cur_m3_ma, cur_m4_ma;
    float   temp_m1_c, temp_m2_c, temp_m3_c, temp_m4_c;
};

class I2cWorker {
public:
    I2cWorker();
    ~I2cWorker();

    bool openI2C();
    void closeI2C();
    bool isOpen() const { return m_fd >= 0; }

    RobotState poll();              // 100ms: 读STATUS + 插值位置 + 模拟传感器

    // 运动命令 (记录目标, 用于插值)
    void cmdArm(qint16 m2_pulse, qint16 m3_pulse, quint16 spd);
    void cmdBase(qint16 m1_pulse, qint16 m4_pulse, quint16 spd);
    void cmdBoth(qint16 m2, qint16 m3, qint16 m1, qint16 m4, quint16 spd);
    void cmdServo(quint8 sv1, quint8 sv2);
    void cmdEstop();
    void cmdEm(int n);                      // 推杆 n 伸出 (1-4)
    void cmdEmOff();                        // 全部推杆缩回
    void cmdBusy(bool on);

private:
    int m_fd;

    // 位置插值状态
    struct MotorTracker {
        float  cur_val;      // 当前显示值 (度/mm)
        float  start_val;    // 命令发出时的值
        float  target_val;   // 目标值
        qint64 start_ms;     // 命令发出时刻 ms
        float  total_ms;     // 预计完成时间 ms
        bool   was_moving;   // 上一帧是否运动中 (检测完成→归静息)
        // 阻尼震荡 (0=无阻尼)
        float  damp_ms;      // 阻尼时长 ms
        qint64 damp_start_ms;
        float  overshoot;
        float  damp_sign;
    };
    MotorTracker m_m1, m_m2, m_m3, m_m4;
    quint8 m_sv1_cmd, m_sv2_cmd;

    void startMove(MotorTracker &t, float cur, float delta, quint16 rpm, float pulsePerUnit, float dampMs);
    float interpolate(MotorTracker &t);
};
