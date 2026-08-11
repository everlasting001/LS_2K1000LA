// ============================================================
// s3_base.ino — 底盘路由器 ESP32-S3
// ============================================================
// 功能: FPGA UART 指令路由 + 大臂电机 M1 本地控制
// MAC:  44:1B:F6:83:E0:80
// 角色:
//   1. 接收 FPGA 通过 UART 发送的 13 字节帧，解析后通过
//      ESP-NOW 仅转发到上臂 S3
//   2. 本地控制大臂 M1 (独占 Serial1, MODBUS-RTU)
//   3. 汇总核心状态，每 20ms 返回 FPGA；传送带缺席固定上报WARN
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

// UART 引脚分配
// M1 (大臂) 独占 Serial1, 使用 Emm 出厂默认地址 1
static const uint8_t M1_ADDR  = 1;
static const uint8_t M1_RX    = 2;
static const uint8_t M1_TX    = 1;

// FPGA UART (Serial2)
static const uint8_t FPGA_RX   = 18;   // ← FPGA P19 router_txd
static const uint8_t FPGA_TX   = 17;   // → FPGA A15 router_rxd

HardwareSerial FPGA_UART(2);

// 将原始通信帧打印到 USB 调试串口（115200）。状态回包会限频，避免刷屏。
static const uint32_t STATUS_LOG_INTERVAL_MS = 5000;
static void debug_hex_frame(const char *label, const uint8_t *data, size_t len) {
    Serial.print(label);
    for (size_t i = 0; i < len; ++i) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX);
        if (i + 1 < len) Serial.print(' ');
    }
    Serial.println();
}

// ============================================================
// 二、状态变量
// ============================================================

// 执行端状态缓存
static StatusPacket arm_status;

// 执行端在线状态跟踪 (500ms 无数据视为离线)
static bool     arm_online        = false;
static uint32_t arm_seen          = 0;     // 最后收到上臂状态的时间

// 本地 M1 电机状态
static uint16_t m1_position       = 0;
static uint8_t  m1_status         = 0;
static uint8_t  base_error        = 0;
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

// 全局急停: 停止本地 M1 + 发送急停到上臂
static void estop_all() {
    emm_stop(Serial1, M1_ADDR);

    CmdPacket c = new_command(CMD_ESTOP);
    c.flags = FLAG_ESTOP;

    send_packet(ARM_MAC, c);
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
        if (v1 < -12000 || v1 > 12000 || speed > 1000)
            base_error |= 0x01;
        else if (!emm_position(Serial1, M1_ADDR, v1, speed, true))
            base_error |= 0x02;
        else
            base_error &= ~0x03;
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

    // bit6(M4)和bit7(推杆)在当前实物中缺席：有意忽略，仅上报黄色WARN。
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
            debug_hex_frame("FPGA -> S3 RX: ", f, sizeof(f));
            if (crc8_ccitt(f, 12) == f[12]) {
                Serial.println("FPGA RX CRC: OK");
                handle_fpga_frame(f);
            } else {
                Serial.println("FPGA RX CRC: FAIL (frame ignored)");
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
        base_error |= 0x02;
    } else {
        m1_status &= 0x7F;
        emm_read_status(Serial1, M1_ADDR, m1_status);
        base_error &= ~0x02;
    }
}

// ============================================================
// 九、状态回复给 FPGA
// ============================================================

// 每20ms向FPGA发送18字节状态帧（17字节数据+CRC8）
// 格式:
//   Byte 0:        0xAA (帧头)
//   Byte 1:        在线标志 (bit0=M2在线, bit1=M3在线, bit5=M1在线, bit6=M4在线)
//   Byte 2–3:      arm_status.pos1 (上臂 M2 位置)
//   Byte 4–5:      arm_status.pos2 (上臂 M3 位置)
//   Byte 6–7:      m1_position (本地 M1 位置)
//   Byte 8–9:      0（传送带缺席）
//   Byte 10–11:    arm_status.aux1 (舵机1 角度)
//   Byte 12–13:    arm_status.aux2 (舵机2 角度)
//   Byte 14:       WARN: bit0传送带缺席 bit1推杆缺席
//   Byte 15:       核心ERROR
//   Byte 16:       DONE: bit0=M1 bit1=M2 bit2=M3 bit5=三轴全到位
//   Byte 17:       CRC-8/CCITT

static void reply_loop() {
    if (millis() - last_reply < 20) return;
    last_reply = millis();

    // 超时检测: 500ms 无数据视为离线
    arm_online      = arm_online      && (millis() - arm_seen      < 500);

    uint8_t r[18] = {
        0xAA,
        (uint8_t)(
            (arm_online      ? 0x06 : 0) |   // bit0=M2, bit1=M3
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
    put16(8,  0);                       // M4缺席
    put16(10, arm_status.aux1);         // 舵机1 角度
    put16(12, arm_status.aux2);         // 舵机2 角度

    r[14] = 0x03; // 两个可选子系统缺席，固定WARN且不阻塞
    r[15] = (base_error & 0x03) | ((arm_status.error & 0x0F) << 2)
          | (!arm_online ? 0x40 : 0);
    r[16] = ((m1_status & 0x08) ? 0x01 : 0)
          | ((arm_status.stat1 & 0x08) ? 0x02 : 0)
          | ((arm_status.stat2 & 0x08) ? 0x04 : 0) | 0x18;
    if ((r[16] & 0x07) == 0x07) r[16] |= 0x20;
    r[17] = crc8_ccitt(r, 17);

    FPGA_UART.write(r, sizeof(r));

    // 内容变化时立即打印；内容不变时每5秒打印一次心跳样本。
    static uint8_t last_logged[18] = {0};
    static bool have_last_logged = false;
    static uint32_t last_log_ms = 0;
    const uint32_t now = millis();
    if (!have_last_logged || memcmp(r, last_logged, sizeof(r)) != 0
            || now - last_log_ms >= STATUS_LOG_INTERVAL_MS) {
        debug_hex_frame("S3 -> FPGA TX: ", r, sizeof(r));
        memcpy(last_logged, r, sizeof(r));
        have_last_logged = true;
        last_log_ms = now;
    }
}

// ============================================================
// 十、初始化和主循环
// ============================================================

void setup() {
    // 初始化调试串口
    Serial.begin(115200);

    // 初始化 M1 电机 UART
    Serial1.begin(115200, SERIAL_8N1, M1_RX, M1_TX);
    delay(100);
    Serial.printf("M1 enable: %s\n", emm_enable(Serial1, M1_ADDR, true) ? "OK" : "FAIL");

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

    // 当前只注册上臂核心执行端；不注册、不等待传送带节点
    add_peer(ARM_MAC);
}

void loop() {
    uart_loop();      // FPGA UART 帧接收与路由
    motor_poll();     // M1 状态轮询
    reply_loop();     // 状态汇总回复 FPGA
    delay(1);         // 让出 CPU
}
