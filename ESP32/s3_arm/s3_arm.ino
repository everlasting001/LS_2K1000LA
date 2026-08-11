// ============================================================
// s3_arm.ino — 上臂执行端 ESP32-S3
// ============================================================
// 功能: 升降电机 M2 + 小臂电机 M3 + 旋转舵机 + 夹爪舵机
// MAC:  44:1B:F6:81:C4:E0
// 角色:
//   1. 通过 ESP-NOW 接收底盘路由器的命令
//   2. 控制 M2 (升降, Serial1) 和 M3 (小臂, Serial2)
//   3. 输出两路 PWM 控制旋转舵机 (GPIO4) 和夹爪舵机 (GPIO5)
//   4. 每 50ms 上报状态
// ============================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include <driver/ledc.h>
#include "protocol.h"
#include "emm_motor.h"

// ============================================================
// 一、硬件配置
// ============================================================

// 底盘路由器 S3 MAC 地址
static uint8_t ROUTER_MAC[6] = {
    0x44, 0x1B, 0xF6, 0x83, 0xE0, 0x80
};

// 两台电机各自独占一个 UART，均使用 Emm 出厂默认地址 1
static const uint8_t M2_ADDR = 1;
static const uint8_t M3_ADDR = 1;

// M2 (升降) 使用 Serial1
static const uint8_t M2_RX = 18;
static const uint8_t M2_TX = 17;

// M3 (小臂) 使用 Serial2
static const uint8_t M3_RX = 16;
static const uint8_t M3_TX = 15;

// 舵机 PWM 引脚
static const uint8_t ROTATE_PIN  = 4;     // 旋转舵机 (270° 数字舵机)
static const uint8_t GRIPPER_PIN = 5;     // 夹爪舵机 (180° 数字舵机)

// 通信超时: 1 秒无命令触发急停
static const uint32_t LINK_TIMEOUT_MS = 1000;

// ============================================================
// 二、状态变量
// ============================================================

// 命令处理 (中断安全: 接收回调写入, 主循环读取)
static volatile bool command_ready = false;
static CmdPacket     pending_command;

// 状态上报
static StatusPacket status_packet;
static uint32_t     last_status       = 0;

// 舵机当前角度
static uint16_t rotate_angle  = 127;      // 旋转舵机: 127° 居中
static uint16_t gripper_angle = 80;       // 夹爪舵机: 80° 张开

// 在线状态
static uint32_t last_valid_command = 0;   // 最后有效命令时间
static uint8_t last_executed_seq = 0;
static bool have_executed_seq = false;

// 电机轮询控制
static uint32_t last_motor_poll = 0;
static bool     poll_m3         = false;  // 交替轮询 M2/M3

// ============================================================
// 三、舵机 PWM 控制 (LEDC 硬件)
// ============================================================

// 舵机占空比计算
//   angle:   目标角度 (0–maximum)
//   maximum: 舵机最大角度 (270 或 180)
// 公式: 脉宽 = 500μs + angle × 2000μs / maximum
//       占空比 = 脉宽 × (2^14 - 1) / 20000μs
static uint32_t servo_duty(uint16_t angle, uint16_t maximum) {
    if (angle > maximum) angle = maximum;

    // 脉宽 (μs): 500–2500
    uint32_t pulse = 500UL + (uint32_t)angle * 2000UL / maximum;

    // 转换为 14 位占空比
    return pulse * ((1UL << 14) - 1) / 20000UL;
}

// 写入舵机角度
static void servo_write(
        ledc_channel_t channel, uint16_t angle, uint16_t maximum)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel,
                  servo_duty(angle, maximum));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// 初始化两路 LEDC PWM (50Hz, 14-bit)
static void servo_setup() {
    // 配置定时器: 50Hz, 14 位分辨率
    ledc_timer_config_t timer = {};
    timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution  = LEDC_TIMER_14_BIT;
    timer.timer_num        = LEDC_TIMER_0;
    timer.freq_hz          = 50;        // 舵机标准 50Hz
    timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    // 配置旋转舵机通道 (CH0 → GPIO4)
    ledc_channel_config_t c = {};
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.timer_sel  = LEDC_TIMER_0;
    c.intr_type  = LEDC_INTR_DISABLE;
    c.duty       = 0;
    c.hpoint     = 0;

    c.channel   = LEDC_CHANNEL_0;
    c.gpio_num  = ROTATE_PIN;
    ledc_channel_config(&c);

    // 配置夹爪舵机通道 (CH1 → GPIO5)
    c.channel   = LEDC_CHANNEL_1;
    c.gpio_num  = GRIPPER_PIN;
    ledc_channel_config(&c);

    // 设置初始角度
    servo_write(LEDC_CHANNEL_0, rotate_angle,  270);   // 旋转舵机 → 127°
    servo_write(LEDC_CHANNEL_1, gripper_angle, 180);   // 夹爪舵机 → 80° (张开)
}

// ============================================================
// 四、急停
// ============================================================

// 立即停止 M2 和 M3
static void emergency_stop() {
    emm_stop(Serial1, M2_ADDR);     // 停止升降
    emm_stop(Serial2, M3_ADDR);     // 停止小臂
}

// ============================================================
// 五、ESP-NOW 通信
// ============================================================

// 接收命令回调: 只接受来自路由器的有效命令包
static void process_receive(
        const uint8_t *mac, const uint8_t *data, int len)
{
    // 只接受来自已知路由器的数据
    if (memcmp(mac, ROUTER_MAC, 6) != 0) return;

    // 长度验证
    if (len != (int)sizeof(CmdPacket)) return;

    const CmdPacket &p = *(const CmdPacket *)data;

    // 校验魔数、版本、CRC
    if (!packet_valid(p, CMD_MAGIC)) return;

    last_valid_command = millis();

    // FPGA/路由层可能重发同一序列号；只回复状态，不重复驱动电机。
    if (have_executed_seq && p.seq == last_executed_seq) {
        status_packet.seq = p.seq;
        return;
    }

    // 缓存命令 (中断安全)
    memcpy(&pending_command, &p, sizeof(p));
    command_ready      = true;
}

// ESP-NOW 接收回调 (兼容 Arduino ESP32 v2 和 v3 API)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void on_receive(
        const esp_now_recv_info_t *info,
        const uint8_t *data, int len)
{
    process_receive(info->src_addr, data, len);
}
#else
static void on_receive(
        const uint8_t *mac, const uint8_t *data, int len)
{
    process_receive(mac, data, len);
}
#endif

// 每 50ms 上报一次状态给路由器
static void send_status() {
    if (millis() - last_status < 50) return;
    last_status = millis();

    status_packet.magic   = STATUS_MAGIC;
    status_packet.version = PROTOCOL_VERSION;
    status_packet.device  = DEV_ARM;
    status_packet.aux1    = rotate_angle;
    status_packet.aux2    = gripper_angle;

    packet_set_crc(status_packet);
    esp_now_send(ROUTER_MAC,
                 (uint8_t *)&status_packet, sizeof(status_packet));
}

// ============================================================
// 六、电机状态轮询 (交错)
// ============================================================

// 每 100ms 交替轮询 M2 和 M3 (每次只查一台，避免总线冲突)
static void poll_motors() {
    if (millis() - last_motor_poll < 100) return;
    last_motor_poll = millis();
    poll_m3 = !poll_m3;     // 翻转标志

    if (poll_m3) {
        // 轮询 M3 (小臂, Serial2)
        if (!emm_read_position(Serial2, M3_ADDR, status_packet.pos2)) {
            status_packet.error |= 0x02;
        } else {
            status_packet.error &= ~0x02;
            emm_read_status(Serial2, M3_ADDR, status_packet.stat2);
        }
    } else {
        // 轮询 M2 (升降, Serial1)
        if (!emm_read_position(Serial1, M2_ADDR, status_packet.pos1)) {
            status_packet.error |= 0x01;
        } else {
            status_packet.error &= ~0x01;
            emm_read_status(Serial1, M2_ADDR, status_packet.stat1);
        }
    }
}

// ============================================================
// 七、初始化和主循环
// ============================================================

void setup() {
    // 初始化调试串口
    Serial.begin(115200);

    // 初始化 M2 和 M3 电机 UART (独立串口)
    Serial1.begin(115200, SERIAL_8N1, M2_RX, M2_TX);
    Serial2.begin(115200, SERIAL_8N1, M3_RX, M3_TX);
    delay(100);
    Serial.printf("M2 enable: %s\n", emm_enable(Serial1, M2_ADDR, true) ? "OK" : "FAIL");
    Serial.printf("M3 enable: %s\n", emm_enable(Serial2, M3_ADDR, true) ? "OK" : "FAIL");

    // 初始化舵机 PWM
    servo_setup();

    // 初始化 ESP-NOW
    memset(&status_packet, 0, sizeof(status_packet));
    WiFi.mode(WIFI_STA);
    Serial.printf("ARM MAC: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(on_receive);

    // 注册路由器
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, ROUTER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    last_valid_command = millis();
}

void loop() {
    // ---- 处理待执行命令 ----
    if (command_ready) {
        // 临界区: 拷贝命令后释放标志
        noInterrupts();
        CmdPacket c = pending_command;
        command_ready = false;
        interrupts();

        // 记录序列号用于状态回复
        status_packet.seq = c.seq;
        last_executed_seq = c.seq;
        have_executed_seq = true;

        if (c.cmd == CMD_ESTOP || (c.flags & FLAG_ESTOP)) {
            // 急停
            emergency_stop();
        }
        else if (c.cmd == CMD_MOVE) {
            // 电机位置移动
            bool relative = c.flags & FLAG_RELATIVE;

            // M2 (升降)
            if (c.flags & FLAG_M2_ENABLE) {
                uint16_t spd = c.speed1 ? c.speed1 : 300;
                if (c.value1 < -192000 || c.value1 > 192000 || spd > 1000) {
                    status_packet.error |= 0x08;
                } else if (!emm_position(Serial1, M2_ADDR, c.value1,
                                  spd, relative)) {
                    status_packet.error |= 0x01;
                } else {
                    status_packet.error &= ~0x09;
                }
            }

            // M3 (小臂)
            if (c.flags & FLAG_M3_ENABLE) {
                uint16_t spd = c.speed2 ? c.speed2 : 300;
                if (c.value2 < -10000 || c.value2 > 10000 || spd > 1000) {
                    status_packet.error |= 0x08;
                } else if (!emm_position(Serial2, M3_ADDR, c.value2,
                                  spd, relative)) {
                    status_packet.error |= 0x02;
                } else {
                    status_packet.error &= ~0x0A;
                }
            }
        }
        else if (c.cmd == CMD_SERVO) {
            // 舵机控制: aux1=旋转角度, aux2=夹爪角度
            // 0xFFFF 表示不更新该通道
            if (c.aux1 != 0xFFFF) {
                if (c.aux1 > 270) {
                    status_packet.error |= 0x08;
                } else {
                    rotate_angle = c.aux1;
                    servo_write(LEDC_CHANNEL_0, rotate_angle, 270);
                    status_packet.error &= ~0x08;
                }
            }
            if (c.aux2 != 0xFFFF) {
                if (c.aux2 > 180) {
                    status_packet.error |= 0x08;
                } else {
                    gripper_angle = c.aux2;
                    servo_write(LEDC_CHANNEL_1, gripper_angle, 180);
                    status_packet.error &= ~0x08;
                }
            }
        }
    }

    // ---- 通信超时检测 ----
    if (millis() - last_valid_command > LINK_TIMEOUT_MS) {
        emergency_stop();
        last_valid_command = millis();
        status_packet.error |= 0x04;
    }

    // ---- 后台任务 ----
    poll_motors();
    send_status();
    delay(1);
}
