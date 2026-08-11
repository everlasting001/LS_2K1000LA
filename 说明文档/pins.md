# GW5AT-60K 与底板引脚映射

> 芯片：Gowin GW5AT-LV60PG484AC1/I0 (Arora V 系列)
> 底板：Loongson 2K1000LA 开发平台

---

## 1. 时钟引脚

### 低速时钟 (50 MHz)

| 信号         | 引脚  | BANK      |
|-------------|-------|-----------|
| CLK_50MHz   | F13   | IOT80A    |

### 高速差分时钟 (200 MHz)

| 信号            | 引脚  |
|----------------|-------|
| F_CLK_200M_P   | D17   |
| F_CLK_200M_N   | C17   |

---

## 2. 复位引脚

| 信号 | 引脚  | BANK    |
|------|-------|---------|
| RST  | D1    | IOL27B  |

---

## 3. 蜂鸣器 PWM

| 信号      | 引脚  |
|----------|-------|
| PWM_beep | A13   |

---

## 4. GPIO 扩展排针 (J14, 2×15)

| 排针序号 | 引脚  | BANK      | 备注           |
|---------|-------|-----------|----------------|
| 1       | —     | —         | GND            |
| 2       | —     | —         | 5V             |
| 3       | —     | —         | GND            |
| 4       | —     | —         | 3.3V           |
| 5       | AA18  | IOB77A    | 上拉到 3.3V     |
| 6       | P14   | IOB146A   |                |
| 7       | W15   | IOB126B   | 上拉到 3.3V     |
| 8       | R16   | IOB142B   |                |
| 9       | V17   | IOB79A    |                |
| 10      | P15   | IOB142A   |                |
| 11      | R19   | IOB113B   |                |
| 12      | N14   | IOB138B   |                |
| 13      | F19   | IOT124A   |                |
| 14      | U18   | IOB75B    |                |
| 15      | E19   | IOT120A   |                |
| 16      | N15   | IOB134B   |                |
| 17      | V18   | IOB81A    |                |
| 18      | P17   | IOB140B   |                |
| 19      | F20   | IOT124B   |                |
| 20      | T18   | IOB144B   | 项目调试串口 DBG_TX |
| 21      | E22   | IOT140A   | 空闲（旧版 EM1，现已迁移到传送带 S3） |
| 22      | R17   | IOB136B   | 空闲（旧版 EM2，现已迁移到传送带 S3） |
| 23      | C18   | IOT115A   | 空闲（旧版 EM3，现已迁移到传送带 S3） |
| 24      | N17   | IOB140A   | 空闲（旧版 EM4，现已迁移到传送带 S3） |
| 25      | C20   | IOT131B   |                |
| 26      | R18   | IOB144A   |                |
| 27      | B16   | IOT97B    |                |
| 28      | Y18   | IOB87A    |                |
| 29      | A15   | IOT99A    | router_rxd：底盘 S3 GPIO17 → FPGA |
| 30      | P19   | IOB113A   | router_txd：FPGA → 底盘 S3 GPIO18 |

---

## 5. PCIe x4 通信

### 差分接收 (RX)

| 信号          | 引脚  |
|--------------|-------|
| PCIE_RX0_P   | B8    |
| PCIE_RX0_N   | A8    |
| PCIE_RX1_P   | D11   |
| PCIE_RX1_N   | C11   |
| PCIE_RX2_P   | B10   |
| PCIE_RX2_N   | A10   |
| PCIE_RX3_P   | D9    |
| PCIE_RX3_N   | C9    |

### 差分发送 (TX)

| 信号          | 引脚  |
|--------------|-------|
| PCIE_TX0_P   | B4    |
| PCIE_TX0_N   | A4    |
| PCIE_TX1_P   | D5    |
| PCIE_TX1_N   | C5    |
| PCIE_TX2_P   | B6    |
| PCIE_TX2_N   | A6    |
| PCIE_TX3_P   | D7    |
| PCIE_TX3_N   | C7    |

### 专用参考时钟 (100 MHz)

| 信号           | 引脚  |
|---------------|-------|
| Q0_REFCLKP_0  | F6    |
| Q0_REFCLKM_0  | E6    |

---

## 6. 板载通用 UART（当前固件未使用）

| 信号 | 引脚 | BANK | 通信对方/方向 |
|---|---|---|---|
| UART_TX | AB20 | IOB85B | FPGA TX → 外部USB-TTL RX |
| UART_RX | T14 | IOB132B | FPGA RX ← 外部USB-TTL TX |

> 当前项目实际调试输出使用 T18 `dbg_txd`，只连接 USB-TTL RX；AB20/T14暂不占用。

---

## 7. I2C 通信 (龙芯 ↔ FPGA)

> FPGA 为 I2C Slave，龙芯为 Master
> 底板直连，I2C-1 总线，地址 0x20

| 信号 | FPGA 引脚 | BANK | 方向 | 对应龙芯信号 |
|------|----------|------|------|-------------|
| I2C_SCL | D20 | IOT131A | inout | LS2K_IIC1_SCL |
| I2C_SDA | C22 | IOT133A | inout | LS2K_IIC1_SDA |

> SCL/SDA 需配置 PULL_MODE=UP，IO_TYPE=LVCMOS33

### 当前项目实际通信引脚

| 本端引脚 | 本端方向 | 通信对方 | 完整连接 | 参数 |
|---|---|---|---|---|
| FPGA P19 / `router_txd` | TX输出 | 底盘S3 GPIO18 / RX | FPGA P19 TX → 底盘S3 GPIO18 RX | 115200, 8N1 |
| FPGA A15 / `router_rxd` | RX输入 | 底盘S3 GPIO17 / TX | FPGA A15 RX ← 底盘S3 GPIO17 TX | 115200, 8N1 |
| FPGA T18 / `dbg_txd` | TX输出 | USB-TTL RX | FPGA T18 TX → USB-TTL RX | 115200, 8N1 |

> FPGA 与底盘 S3 必须共地。P19/A15 均为 3.3V 逻辑，不可接 5V TTL。

### 三块 ESP32-S3 执行端引脚

| 控制板本端 | TX连接的对方 | RX连接的对方 | 功能 |
|---|---|---|---|
| 底盘S3 GPIO1 TX / GPIO2 RX | GPIO1 TX → 大臂M1 RX | GPIO2 RX ← 大臂M1 TX | 大臂M1，Emm地址1 |
| 底盘S3 GPIO17 TX / GPIO18 RX | GPIO17 TX → FPGA A15 RX | GPIO18 RX ← FPGA P19 TX | FPGA路由串口 |
| 上臂S3 GPIO17 TX / GPIO18 RX | GPIO17 TX → 升降M2 RX | GPIO18 RX ← 升降M2 TX | 升降M2，Emm地址1 |
| 上臂S3 GPIO15 TX / GPIO16 RX | GPIO15 TX → 小臂M3 RX | GPIO16 RX ← 小臂M3 TX | 小臂M3，Emm地址1 |
| 上臂S3 GPIO4 | — | — | 旋转舵机信号，0～270°，复位127° |
| 上臂S3 GPIO5 | — | — | 夹爪舵机信号，0～180° |
| 传送带S3 GPIO17 TX / GPIO18 RX | GPIO17 TX → 传送带M4 RX | GPIO18 RX ← 传送带M4 TX | 传送带M4，Emm地址1 |
| 传送带S3 GPIO7/8/9/10 | — | — | 电磁铁1～4驱动输入，高电平推出 |

四台 Emm 均独占各自 UART，因此可以全部使用默认地址 `1`。

---

## 8. SPI 通信 (龙芯 ↔ FPGA)

> FPGA 为 SPI Slave，龙芯为 Master
> 底板直连，无需飞线

| 信号 | FPGA 引脚 | BANK | 方向 | 对应龙芯信号 |
|------|----------|------|------|-------------|
| SPI_SCLK | E21 | IOT138A | input | LS2H_SPI_SCK |
| SPI_MOSI | U8 | IOR11A | input | LS2H_SPI_SDO |
| SPI_MISO | D21 | IOT138B | output | LS2H_SPI_SDI |
| SPI_CSN | B22 | IOT133B | input | LS2H_SPI_CSN1 |

备用 CSN: B21 (CSN2), A21 (CSN3)

---

## 8. 拨码开关 (8 位)

| 开关   | 引脚  |
|-------|-------|
| sw0   | M2    |
| sw1   | K3    |
| sw2   | K4    |
| sw3   | L5    |
| sw4   | M3    |
| sw5   | L3    |
| sw6   | J4    |
| sw7   | L4    |

---

## 8. LED 指示灯 (8 位)

| LED   | 引脚  |
|-------|-------|
| led0  | M1    |
| led1  | L1    |
| led2  | J6    |
| led3  | K6    |
| led4  | N4    |
| led5  | N3    |
| led6  | L6    |
| led7  | P5    |

---

## 9. 四位八段数码管

### 段选信号

| 段选    | 引脚  | 说明  |
|--------|-------|-------|
| seg0   | K16   | a     |
| seg1   | L16   | b     |
| seg2   | H20   | c     |
| seg3   | G20   | d     |
| seg4   | J22   | e     |
| seg5   | K17   | f     |
| seg6   | H22   | g     |
| seg7   | M15   | DP    |

### 位选信号 (共阳极)

| 位选   | 引脚  | 说明    |
|--------|-------|---------|
| dig0   | J17   | 第1位   |
| dig1   | L13   | 第2位   |
| dig2   | M22   | 第3位   |
| dig3   | N22   | 第4位   |

---

## 10. 矩阵键盘 (4×4)

> **原理说明：**
> - **行 ROW[0:3]** → 输出，默认高电平，FPGA 逐行主动拉低扫描
> - **列 COL[0:3]** → 输入，默认高电平（上拉），按键按下时被拉低，FPGA 检测

### 行信号 (输出)

| 信号   | 引脚  |
|-------|-------|
| row0  | A1    |
| row1  | B2    |
| row2  | C2    |
| row3  | D2    |

### 列信号 (输入, 上拉)

| 信号   | 引脚  |
|-------|-------|
| col0  | E2    |
| col1  | G1    |
| col2  | E1    |
| col3  | F1    |

---

## 11. 旋转编码器 (2 个)

### 编码器 1

| 信号       | 引脚  |
|-----------|-------|
| PulseA_1  | U15   |
| PulseB_1  | U16   |

### 编码器 2

| 信号       | 引脚  |
|-----------|-------|
| PulseA_2  | R14   |
| PulseB_2  | N13   |

---

## 附录：引脚速查表 (按引脚排序)

| 引脚  | 功能                              |
|-------|-----------------------------------|
| A1    | 矩阵键盘 row0                      |
| A4    | PCIE_TX0_N                        |
| A6    | PCIE_TX2_N                        |
| A8    | PCIE_RX0_N                        |
| A10   | PCIE_RX2_N                        |
| A13   | 蜂鸣器 PWM_beep                   |
| A15   | GPIO 排针 29，router_rxd（←底盘S3 GPIO17） |
| AA18  | GPIO 排针 5 (IOB77A, 上拉 3.3V)   |
| AB20  | 通用UART_TX → 外部USB-TTL RX（当前未使用） |
| B2    | 矩阵键盘 row1                      |
| B4    | PCIE_TX0_P                        |
| B6    | PCIE_TX2_P                        |
| B8    | PCIE_RX0_P                        |
| B10   | PCIE_RX2_P                        |
| B16   | GPIO 排针 27 (IOT97B)             |
| C2    | 矩阵键盘 row2                      |
| C5    | PCIE_TX1_N                        |
| C7    | PCIE_TX3_N                        |
| C9    | PCIE_RX3_N                        |
| C11   | PCIE_RX1_N                        |
| C17   | F_CLK_200M_N                      |
| C18   | GPIO 排针 23，空闲（旧版 EM3）     |
| C20   | GPIO 排针 25 (IOT131B)            |
| D1    | RST (IOL27B)                      |
| D2    | 矩阵键盘 row3                      |
| D5    | PCIE_TX1_P                        |
| D7    | PCIE_TX3_P                        |
| D9    | PCIE_RX3_P                        |
| D11   | PCIE_RX1_P                        |
| D17   | F_CLK_200M_P                      |
| E1    | 矩阵键盘 col2                      |
| E2    | 矩阵键盘 col0                      |
| E6    | Q0_REFCLKM_0                      |
| E19   | GPIO 排针 15 (IOT120A)            |
| E22   | GPIO 排针 21，空闲（旧版 EM1）     |
| F1    | 矩阵键盘 col3                      |
| F6    | Q0_REFCLKP_0                      |
| F13   | CLK_50MHz (IOT80A)                |
| F19   | GPIO 排针 13 (IOT124A)            |
| F20   | GPIO 排针 19 (IOT124B)            |
| G1    | 矩阵键盘 col1                      |
| G20   | 数码管 seg3 (d)                    |
| H20   | 数码管 seg2 (c)                    |
| H22   | 数码管 seg6 (g)                    |
| J4    | 拨码开关 sw6                       |
| J6    | LED led2                          |
| J17   | 数码管 dig0 (第1位)                |
| J22   | 数码管 seg4 (e)                    |
| K3    | 拨码开关 sw1                       |
| K4    | 拨码开关 sw2                       |
| K6    | LED led3                          |
| K16   | 数码管 seg0 (a)                    |
| K17   | 数码管 seg5 (f)                    |
| L1    | LED led1                          |
| L3    | 拨码开关 sw5                       |
| L4    | 拨码开关 sw7                       |
| L5    | 拨码开关 sw3                       |
| L6    | LED led6                          |
| L13   | 数码管 dig1 (第2位)                |
| L16   | 数码管 seg1 (b)                    |
| M1    | LED led0                          |
| M2    | 拨码开关 sw0                       |
| M3    | 拨码开关 sw4                       |
| M15   | 数码管 seg7 (DP)                   |
| M22   | 数码管 dig2 (第3位)                |
| N3    | LED led5                          |
| N4    | LED led4                          |
| N13   | 旋转编码器2 PulseB_2               |
| N14   | GPIO 排针 12 (IOB138B)            |
| N15   | GPIO 排针 16 (IOB134B)            |
| N17   | GPIO 排针 24，空闲（旧版 EM4）     |
| N22   | 数码管 dig3 (第4位)                |
| P5    | LED led7                          |
| P14   | GPIO 排针 6 (IOB146A)             |
| P15   | GPIO 排针 10 (IOB142A)            |
| P17   | GPIO 排针 18 (IOB140B)            |
| P19   | GPIO 排针 30，router_txd（→底盘S3 GPIO18） |
| R14   | 旋转编码器2 PulseA_2               |
| R16   | GPIO 排针 8 (IOB142B)             |
| R17   | GPIO 排针 22，空闲（旧版 EM2）     |
| R18   | GPIO 排针 26 (IOB144A)            |
| R19   | GPIO 排针 11 (IOB113B)            |
| T14   | 通用UART_RX ← 外部USB-TTL TX（当前未使用） |
| T18   | GPIO排针20，FPGA dbg_txd TX → USB-TTL RX |
| U15   | 旋转编码器1 PulseA_1               |
| U16   | 旋转编码器1 PulseB_1               |
| U18   | GPIO 排针 14 (IOB75B)             |
| V17   | GPIO 排针 9 (IOB81A)              |
| V18   | GPIO 排针 17 (IOB81A)             |
| W15   | GPIO 排针 7 (IOB126B, 上拉 3.3V)  |
| Y18   | GPIO 排针 28 (IOB87A)             |
