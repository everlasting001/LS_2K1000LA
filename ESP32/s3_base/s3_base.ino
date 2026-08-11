// ============================================================
// s3_base.ino — 底盘路由器 ESP32-S3
// ============================================================
// 功能: FPGA UART 指令路由 + 大臂电机 M1 本地控制
// MAC:  44:1B:F6:83:E0:80
// 角色:
//   1. 接收 FPGA 通过 UART 发送的 13 字节帧，解析后通过
//      ESP-NOW 转发到上臂 S3 和传送带 S3
//   2. 本地控制大臂 M1 (独占 Serial1, MODBUS-RTU)
//   3. 汇总各执行端状态，每 20ms 通过 UART 返回给 FPGA (17 字节帧)
// ============================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include "protocol.h"
#include "emm_motor.h"

// ============================================================
// 一、硬件配置
// ============================================================

// 上臂 S3 MAC 地址
static uint8_t ARM_MAC[6] = {
    0x44, 0x1B, 0xF6, 0x81, 0xC4, 0xE0
};

// 传送带 S3 MAC 地址
static uint8_t CONVEYOR_MAC[6] = {
    0x44, 0x1B, 0xF6, 0x83, 0xDF, 0xD8
};

// UART 引脚分配
// M1 (大臂) 独占 Serial1, 使用 Emm 出厂默认地址 1
static const uint8_t M1_ADDR  = 1;
static const uint8_t M1_RX    = 2;
static const uint8_t M1_TX    = 1;

// FPGA UART (Serial2)
static const uint8_t FPGA_RX   = 20;   // ← FPGA P19 router_txd
static const uint8_t FPGA_TX   = 21;   // → FPGA A15 router_rxd

HardwareSerial FPGA_UART(2);

// ============================================================
// 二、状态变量
// ============================================================

// 执行端状态缓存
static StatusPacket arm_status;
static StatusPacket conveyor_status;

// 执行端在线状态跟踪 (500ms 无数据视为离线)
static bool     arm_online        = false;
static bool     conveyor_online   = false;
static uint32_t arm_seen          = 0;     // 最后收到上臂状态的时间
static uint32_t conveyor_seen     = 0;     // 最后收到传送带状态的时间

// 本地 M1 电机状态
static uint16_t m1_position       = 0;
static uint8_t  m1_status         = 0;
static uint32_t last_m1_poll      = 0;     // 上一次轮询 M1 的时间

// 通信控制
static uint32_t last_reply        = 0;     // 上一次回复 FPGA 的时间
static uint8_t  sequence_number   = 0;     // ESP-NOW 命令序列号 (递增)

// ============================================================
// 三、ESP-NOW 通信函数
// ============================================================

// 检查 MAC 地址是否已配置 (非全 FF)
static bool mac_configured(const uint8_t *mac) {
    for (uint8_t i = 0; i < 6; i++) {
        if (mac[i] != 0xFF) return true;
    }
    return false;
}

// 添加 ESP-NOW 对等设备
static void add_peer(const uint8_t *mac) {
    if (!mac_configured(mac)) return;

    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = 0;          // 使用当前 WiFi 信道
    p.encrypt = false;      // 不加密 (室内近距通信)
    esp_now_add_peer(&p);
}

// 构造新命令包 (自动填入魔数、版本号、递增序列号)
static CmdPacket new_command(uint8_t cmd) {
    CmdPacket c = {};
    c.magic    = CMD_MAGIC;
    c.version  = PROTOCOL_VERSION;
    c.cmd      = cmd;
    c.seq      = ++sequence_number;
    c.aux1     = 0xFFFF;       // 默认值: 无操作
    c.aux2     = 0xFFFF;
    return c;
}

// 发送命令包 (自动计算 CRC)
static void send_packet(const uint8_t *mac, CmdPacket &c) {
    if (!mac_configured(mac)) return;
    packet_set_crc(c);
    esp_now_send(mac, (uint8_t *)&c, sizeof(c));
}

// ============================================================
// 四、ESP-NOW 接收回调
// ============================================================

// 处理收到的状态包: 验证合法性后按设备 ID 分类缓存
static void process_receive(
        const uint8_t *mac, const uint8_t *data, int len)
{
    // 长度必须是 StatusPacket 大小
    if (len != (int)sizeof(StatusPacket)) return;

    const StatusPacket &p = *(const StatusPacket *)data;

    // 校验魔数、版本、CRC
    if (!packet_valid(p, STATUS_MAGIC)) return;

    // 按 MAC 地址匹配设备
    if (!memcmp(mac, ARM_MAC, 6) && p.device == DEV_ARM) {
        memcpy(&arm_status, &p, sizeof(p));
        arm_online = true;
        arm_seen   = millis();
    }
    else if (!memcmp(mac, CONVEYOR_MAC, 6) && p.device == DEV_CONVEYOR) {
        memcpy(&conveyor_status, &p, sizeof(p));
        conveyor_online = true;
        conveyor_seen   = millis();
    }
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

// ============================================================
// 五、急停
// ============================================================

// 全局急停: 停止本地 M1 + 广播急停到上臂和传送带
static void estop_all() {
    emm_stop(Serial1, M1_ADDR);

    CmdPacket c = new_command(CMD_ESTOP);
    c.flags = FLAG_ESTOP;

    send_packet(ARM_MAC, c);
    send_packet(CONVEYOR_MAC, c);
}

// ============================================================
// 六、FPGA 帧解析与命令路由
// ============================================================

// FPGA 帧格式 (固定 13 字节):
//   Byte 0:       0xAA (帧头)
//   Byte 1:       flags (bit1=M2, bit2=M3, bit3=舵机, bit4=急停,
//                         bit5=M1, bit6=M4, bit7=电磁铁)
//   Byte 2–5:     value1 (int32 LE) — M1/M2/M3/舵机1 目标值
//   Byte 6–9:     value2 (int32 LE) — M3辅助/舵机2/M4目标/电磁铁参数
//   Byte 10–11:   param (uint16 LE) — 速度/时长参数
//   Byte 12:      CRC-8/CCITT (覆盖 Byte 0–11)

static void handle_fpga_frame(const uint8_t *f) {
    uint8_t flags = f[1];

    // 解析 value1 (LE int32)
    int32_t v1 = (int32_t)(
          (uint32_t)f[2]
        | ((uint32_t)f[3] << 8)
        | ((uint32_t)f[4] << 16)
        | ((uint32_t)f[5] << 24));

    // 解析 value2 (LE int32)
    int32_t v2 = (int32_t)(
          (uint32_t)f[6]
        | ((uint32_t)f[7] << 8)
        | ((uint32_t)f[8] << 16)
        | ((uint32_t)f[9] << 24));

    // 解析速度/时长参数 (LE uint16)
    uint16_t param = (uint16_t)f[10] | ((uint16_t)f[11] << 8);

    // ---- 急停 (bit4) ----
    if (flags & 0x10) {
        estop_all();
        return;
    }

    // ---- 大臂 M1 (bit5) — 本地控制 ----
    if (flags & 0x20) {
        uint16_t speed = param ? param : 300;  // 默认 300 RPM
        emm_position(Serial1, M1_ADDR, v1, speed, true);
    }

    // ---- 上臂 M2/M3 (bit1, bit2) — 转发到上臂 S3 ----
    if (flags & 0x06) {
        CmdPacket c = new_command(CMD_MOVE);
        c.value1 = v1;
        c.value2 = v2;
        c.speed1 = param ? param : 300;
        c.speed2 = c.speed1;
        c.flags  = FLAG_RELATIVE;       // 默认相对运动

        if (flags & 0x02) c.flags |= FLAG_M2_ENABLE;  // M2
        if (flags & 0x04) c.flags |= FLAG_M3_ENABLE;  // M3

        send_packet(ARM_MAC, c);
    }

    // ---- 舵机 (bit3) — 转发到上臂 S3 ----
    if (flags & 0x08) {
        CmdPacket c = new_command(CMD_SERVO);
        c.aux1 = (uint16_t)v1;      // 旋转舵机角度
        c.aux2 = (uint16_t)v2;      // 夹爪舵机角度
        send_packet(ARM_MAC, c);
    }

    // ---- 传送带 M4 (bit6) — 转发到传送带 S3 ----
    if (flags & 0x40) {
        CmdPacket c = new_command(CMD_MOVE);
        c.value1 = v2;                          // M4 目标脉冲
        c.speed1 = param ? param : 300;
        c.flags  = FLAG_RELATIVE | FLAG_M2_ENABLE;  // 复用 M2_ENABLE 标志
        send_packet(CONVEYOR_MAC, c);
    }

    // ---- 电磁铁 (bit7) — 转发到传送带 S3 ----
    if (flags & 0x80) {
        CmdPacket c = new_command(CMD_SOLENOID);
        c.aux1   = (uint8_t)v1 & 0x0F;    // one-hot 掩码
        c.aux2   = (uint8_t)v2;           // 动作时长 (10ms 单位)
        c.speed2 = param;                 // 冷却时长 (10ms 单位)
        send_packet(CONVEYOR_MAC, c);
    }
}

// ============================================================
// 七、FPGA UART 接收状态机
// ============================================================

// UART 帧接收状态机: 等待帧头 0xAA → 收集 13 字节 → CRC 校验 → 处理
static void uart_loop() {
    static uint8_t f[13];       // 帧缓冲区
    static uint8_t index = 0;   // 当前字节位置

    while (FPGA_UART.available()) {
        uint8_t b = (uint8_t)FPGA_UART.read();

        // 等待帧头 0xAA
        if (index == 0 && b != 0xAA) continue;

        f[index++] = b;

        // 收满 13 字节，校验 CRC-8
        if (index == 13) {
            if (crc8_ccitt(f, 12) == f[12]) {
                handle_fpga_frame(f);
            }
            index = 0;      // 重置，等待下一帧
        }
    }
}

// ============================================================
// 八、M1 电机状态轮询
// ============================================================

// 每 100ms 轮询一次 M1 位置和状态
static void motor_poll() {
    if (millis() - last_m1_poll < 100) return;
    last_m1_poll = millis();

    if (!emm_read_position(Serial1, M1_ADDR, m1_position)) {
        // 读取失败，设置错误标志
        m1_status |= 0x80;
    } else {
        m1_status &= 0x7F;
        emm_read_status(Serial1, M1_ADDR, m1_status);
    }
}

// ============================================================
// 九、状态回复给 FPGA
// ============================================================

// 每 20ms 向 FPGA 发送 17 字节状态帧 (16 字节数据 + CRC8)
// 格式:
//   Byte 0:        0xAA (帧头)
//   Byte 1:        在线标志 (bit0=M2在线, bit1=M3在线, bit5=M1在线, bit6=M4在线)
//   Byte 2–3:      arm_status.pos1 (上臂 M2 位置)
//   Byte 4–5:      arm_status.pos2 (上臂 M3 位置)
//   Byte 6–7:      m1_position (本地 M1 位置)
//   Byte 8–9:      conveyor_status.pos1 (传送带 M4 位置)
//   Byte 10–11:    arm_status.aux1 (舵机1 角度)
//   Byte 12–13:    arm_status.aux2 (舵机2 角度)
//   Byte 14:       conveyor_status.aux1 (电磁铁掩码)
//   Byte 15:       错误码 (低4位=上臂, 高4位=传送带)
//   Byte 16:       CRC-8/CCITT (覆盖 Byte 0–15)

static void reply_loop() {
    if (millis() - last_reply < 20) return;
    last_reply = millis();

    // 超时检测: 500ms 无数据视为离线
    arm_online      = arm_online      && (millis() - arm_seen      < 500);
    conveyor_online = conveyor_online && (millis() - conveyor_seen < 500);

    // 构造 17 字节状态帧
    uint8_t r[17] = {
        0xAA,
        (uint8_t)(
            (arm_online      ? 0x06 : 0) |   // bit0=M2, bit1=M3
            (conveyor_online ? 0x40 : 0) |   // bit6=M4
            0x20                              // bit5=M1 (本地总是在线)
        )
    };

    // 辅助函数: 写入 16 位值 (小端序)
    auto put16 = [&](uint8_t i, uint16_t v) {
        r[i]     = v;
        r[i + 1] = v >> 8;
    };

    put16(2,  arm_status.pos1);         // M2 位置
    put16(4,  arm_status.pos2);         // M3 位置
    put16(6,  m1_position);             // M1 位置 (本地)
    put16(8,  conveyor_status.pos1);    // M4 位置
    put16(10, arm_status.aux1);         // 舵机1 角度
    put16(12, arm_status.aux2);         // 舵机2 角度

    r[14] = conveyor_status.aux1;       // 电磁铁掩码

    // 错误码: 低 4 位=上臂, 高 4 位=传送带
    r[15] = (arm_status.error      & 0x0F)
          | ((conveyor_status.error & 0x0F) << 4);

    // CRC-8 校验
    r[16] = crc8_ccitt(r, 16);

    FPGA_UART.write(r, sizeof(r));
}

// ============================================================
// 十、初始化和主循环
// ============================================================

void setup() {
    // 初始化调试串口
    Serial.begin(115200);

    // 初始化 M1 电机 UART
    Serial1.begin(115200, SERIAL_8N1, M1_RX, M1_TX);

    // 初始化 FPGA UART
    FPGA_UART.begin(115200, SERIAL_8N1, FPGA_RX, FPGA_TX);

    // 初始化 ESP-NOW
    WiFi.mode(WIFI_STA);
    Serial.printf("BASE ROUTER MAC: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(on_receive);

    // 注册两个执行端
    add_peer(ARM_MAC);
    add_peer(CONVEYOR_MAC);
}

void loop() {
    uart_loop();      // FPGA UART 帧接收与路由
    motor_poll();     // M1 状态轮询
    reply_loop();     // 状态汇总回复 FPGA
    delay(1);         // 让出 CPU
}
