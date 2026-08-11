低速时钟引脚(50MHz)：
F13 CLK_50MHz IOT80A

高速差分时钟(200MHz)：
D17 F_CLK_200M_P
C17 F_CLK_200M_N

复位引脚RST：
D1 RST IOL27B

蜂鸣器PWM：
A13 PWM_beep

能用的引出的GPIO引脚（J14 2*15）：
    1 GND
    2 5V
    3 GND
    4 3.3V
    5 AA18(上拉到3.3V) IOB77A
    6 P14 IOB146A
    7 W15(上拉到3.3V) IOB126B
    8 R16 IOB142B
    9 V17 IOB79A
    10 P15 IOB142A
    11 R19 IOB113B
    12 N14 IOB138B
    13 F19 IOT124A
    14 U18 IOB75B
    15 E19 IOT120A
    16 N15 IOB134B
    17 V18 IOB81A
    18 P17 IOB140B
    19 F20 IOT124B
    20 T18 IOB144B
    21 E22 IOT140A
    22 R17 IOB136B
    23 C18 IOT115A
    24 N17 IOB140A
    25 C20 IOT131B
    26 R18 IOB144A
    27 B16 IOT97B
    28 Y18 IOB87A
    29 A15 IOT99A
    30 P19 IOB113A

PCIE x4 通信
差分对信号：
    PCIE_RX0_P B8
    PCIE_RX0_N A8
    PCIE_RX1_P D11
    PCIE_RX1_N C11
    PCIE_RX2_P B10
    PCIE_RX2_N A10
    PCIE_RX3_P D9
    PCIE_RX3_N C9
    PCIE_TX0_P B4
    PCIE_TX0_N A4
    PCIE_TX1_P D5
    PCIE_TX1_N C5
    PCIE_TX2_P B6
    PCIE_TX2_N A6
    PCIE_TX3_P D7
    PCIE_TX3_N C7
专用100MHz时钟：
    Q0_REFCLKP_0 F6
    Q0_REFCLKM_0 E6

集成的uart通信(用于串口调试)：
    UART_TX IOB85B AB20
    UART_RX IOB132B T14

八个开关：
sw0 M2
sw1 K3 
sw2 K4
sw3 L5
sw4 M3
sw5 L3
sw6 J4
sw7 L4

八个LED：
led0 M1
led1 L1
led2 J6
led3 K6
led4 N4
led5 N3
led6 L6
led7 P5

四位八段数码管：
seg0 K16 
seg1 L16
seg2 H20
seg3 G20
seg4 J22
seg5 K17
seg6 H22
seg7 M15 (DP)
dig0 J17
dig1 L13
dig2 M22
dig3 N22

矩阵键盘引脚：
信号	方向	默认电平	驱动方式
行 ROW[0:3]	输出	高电平	FPGA 主动拉低，逐行扫描
列 COL[0:3]	输入	高电平（上拉）	按键按下时被拉低，FPGA 检测

为什么列必须配置为输入且有上拉？
    列线上有上拉电阻：无按键时，列线保持高电平。
    按键按下时：行与列导通，若该行被拉低，则对应列也被拉低。
    FPGA 的工作：被动读取列电平，检测变化，不需要主动驱动列。

为什么行必须配置为输出？
    矩阵键盘需要轮流给每行施加低电平（其他行高电平），才能逐行扫描。
    FPGA 必须主动控制行线的电平（通过 GPIO 输出寄存器）。
    如果行也设为输入，则无法驱动行线，扫描无法进行。
row0 A1
row1 B2
row2 C2
row3 D2
col0 E2
col1 G1
col2 E1
col3 F1

两个旋转编码器：
    编码器1：
        A相 PulseA_1 U15
        B相 PulseB_1 U16
    编码器2：
        A相 PulseA_2 R14
        B相 PulseB_2 N13