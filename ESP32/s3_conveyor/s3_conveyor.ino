// ============================================================
// s3_conveyor.ino — 传送带执行端 ESP32-S3
// ============================================================
// 功能: 传送带电机 M4 + 四路推拉电磁铁
// MAC:  44:1B:F6:83:DF:D8
// 角色:
//   1. 通过 ESP-NOW 接收底盘路由器的命令
//   2. 控制 M4 (传送带, Serial1, MODBUS-RTU)
//   3. 控制四路电磁铁 GPIO7–10 (高电平推出)
//   4. 电磁铁安全: one-hot 互锁 + 动作时长上限 + 冷却间隔
//   5. 每 50ms 上报状态
// ============================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include "protocol.h"
#include "emm_motor.h"

// ============================================================
// 一、硬件配置
// ============================================================

// 底盘路由器 S3 MAC 地址
static uint8_t ROUTER_MAC[6] = {
    0x44, 0x1B, 0xF6, 0x83, 0xE0, 0x80
};

// M4 (传送带) 独占 Serial1，使用 Emm 出厂默认地址 1
static const uint8_t M4_ADDR = 1;
static const uint8_t M4_RX   = 18;
static const uint8_t M4_TX   = 17;

// 电磁铁引脚 (GPIO7–10, 高电平有效)
static const uint8_t EM_PINS[4] = { 7, 8, 9, 10 };

// 通信超时: 1 秒无命令触发急停
static const uint32_t LINK_TIMEOUT_MS = 1000;

// ============================================================
// 二、电磁铁状态机
// ============================================================

// 电磁铁状态枚举
enum EmState : uint8_t {
    EM_IDLE     = 0,    // 空闲, 可接受新命令
    EM_ACTIVE   = 1,    // 激活中, 倒计时推出时长
    EM_COOLDOWN = 2     // 冷却中, 禁止触发
};

// ============================================================
// 三、状态变量
// ============================================================

// 命令处理 (中断安全)
static volatile bool command_ready = false;
static CmdPacket     pending_command;

// 状态上报
static StatusPacket status_packet;
static uint32_t     last_status = 0;

// 电磁铁状态
static EmState  em_state    = EM_IDLE;
static uint8_t  em_mask     = 0;        // 当前激活的电磁铁 one-hot 掩码
static uint32_t em_deadline = 0;        // 状态切换截止时间

// 在线状态
static uint32_t last_valid_command = 0; // 最后有效命令时间

// 电机轮询控制
static uint32_t last_motor_poll = 0;

// ============================================================
// 四、电磁铁控制函数
// ============================================================

// 关闭所有电磁铁
static void all_solenoids_off() {
    for (uint8_t pin : EM_PINS) {
        digitalWrite(pin, LOW);
    }
    em_mask = 0;
}

// 急停: 停止 M4 + 关闭所有电磁铁
static void emergency_stop() {
    emm_stop(Serial1, M4_ADDR);
    all_solenoids_off();
    em_state = EM_IDLE;
}

// 启动指定电磁铁 (one-hot 互锁)
//   mask:          4 位 one-hot 掩码 (bit0–bit3)
//   active_10ms:   动作持续时长 (单位: 10ms, 默认 20 → 200ms)
//   cooldown_10ms: 冷却时长 (单位: 10ms, 默认 50 → 500ms)
// 返回: true=成功启动, false=当前忙/掩码非法
static bool start_solenoid(
        uint8_t mask, uint8_t active_10ms, uint16_t cooldown_10ms)
{
    // 状态检查: 必须空闲
    if (em_state != EM_IDLE) return false;

    // 掩码检查: 必须为合法 one-hot
    if (!valid_one_hot4(mask)) return false;

    // 先关闭所有，再打开目标
    all_solenoids_off();
    em_mask = mask & 0x0F;

    // 逐位设置 GPIO
    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(EM_PINS[i],
                     (em_mask & (1 << i)) ? HIGH : LOW);
    }

    // 计算动作时长 (单位: ms, 默认 200ms)
    uint32_t active = (active_10ms ? active_10ms : 20) * 10UL;

    // 安全上限: 最长 2000ms
    if (active > 2000UL) active = 2000UL;

    em_deadline = millis() + active;

    // 冷却时长 (单位: 10ms, 上限 1000 → 10s)
    status_packet.speed2 = (cooldown_10ms > 1000) ? 1000 : cooldown_10ms;

    em_state = EM_ACTIVE;
    return true;
}

// 电磁铁状态机更新 (每循环调用)
//   IDLE       → 无操作
//   ACTIVE     → 计时到 → 关闭 → 进入 COOLDOWN 或 IDLE
//   COOLDOWN   → 计时到 → 回到 IDLE
static void update_solenoids() {
    // 空闲状态或计时未到则跳过
    if (em_state == EM_IDLE) return;
    if ((int32_t)(millis() - em_deadline) < 0) return;

    if (em_state == EM_ACTIVE) {
        // 动作完成，关闭电磁铁
        all_solenoids_off();

        // 计算冷却时间
        uint32_t cool = (uint32_t)status_packet.speed2 * 10UL;
        if (cool) {
            em_state    = EM_COOLDOWN;
            em_deadline = millis() + cool;
        } else {
            em_state = EM_IDLE;
        }
    } else {
        // 冷却完成
        em_state = EM_IDLE;
    }
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

    // 缓存命令 (中断安全)
    memcpy(&pending_command, &p, sizeof(p));
    command_ready      = true;
    last_valid_command = millis();
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
    status_packet.device  = DEV_CONVEYOR;
    status_packet.aux1    = em_mask;          // 当前激活的电磁铁掩码
    status_packet.aux2    = (uint8_t)em_state; // 电磁铁状态

    packet_set_crc(status_packet);
    esp_now_send(ROUTER_MAC,
                 (uint8_t *)&status_packet, sizeof(status_packet));
}

// ============================================================
// 六、M4 电机状态轮询
// ============================================================

// 每 100ms 轮询一次 M4 位置和状态
static void poll_motor() {
    if (millis() - last_motor_poll < 100) return;
    last_motor_poll = millis();

    if (!emm_read_position(Serial1, M4_ADDR, status_packet.pos1)) {
        status_packet.error |= 0x01;
    } else {
        status_packet.error &= ~0x01;
        emm_read_status(Serial1, M4_ADDR, status_packet.stat1);
    }
}

// ============================================================
// 七、初始化和主循环
// ============================================================

void setup() {
    // 初始化调试串口
    Serial.begin(115200);

    // 初始化电磁铁 GPIO (默认全部低电平)
    for (uint8_t pin : EM_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // 初始化 M4 电机 UART
    Serial1.begin(115200, SERIAL_8N1, M4_RX, M4_TX);

    // 初始化状态包
    memset(&status_packet, 0, sizeof(status_packet));

    // 初始化 ESP-NOW
    WiFi.mode(WIFI_STA);
    Serial.printf("CONVEYOR MAC: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed; outputs remain OFF");
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

        if (c.cmd == CMD_ESTOP || (c.flags & FLAG_ESTOP)) {
            // 急停
            emergency_stop();
        }
        else if (c.cmd == CMD_MOVE) {
            // M4 传送带位置移动
            uint16_t spd = c.speed1 ? c.speed1 : 300;
            if (!emm_position(Serial1, M4_ADDR, c.value1,
                              spd, c.flags & FLAG_RELATIVE)) {
                status_packet.error |= 0x01;
            }
        }
        else if (c.cmd == CMD_SOLENOID) {
            // 电磁铁控制
            //   aux1:   4 位 one-hot 掩码
            //   aux2:   动作时长 (10ms 单位)
            //   speed2: 冷却时长 (10ms 单位)
            if (!start_solenoid(c.aux1, c.aux2, c.speed2)) {
                status_packet.error |= 0x02;
            }
        }
    }

    // ---- 电磁铁状态机更新 ----
    update_solenoids();

    // ---- 通信超时检测 ----
    if (millis() - last_valid_command > LINK_TIMEOUT_MS) {
        emergency_stop();
        last_valid_command = millis();
        status_packet.error |= 0x04;
    }

    // ---- 后台任务 ----
    poll_motor();
    send_status();
    delay(1);
}
