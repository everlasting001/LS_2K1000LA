//******************************************************************************
// top.v — SCARA 四轴机械臂 FPGA 主控 (v5: S3 路由器协议)
//
// 架构:
//   龙芯 ←I2C→ FPGA ←UART→ 底盘S3路由器(M1)
//                              ├ESP-NOW→ 上臂S3(M2+M3+舵机)
//                              └ESP-NOW→ 传送带S3(M4+四路电磁铁)
//
// S3 路由协议:
//   FPGA→S3: [AA] [flags] [val1_lo] [val1_hi] [val2_lo] [val2_hi] [param_lo] [param_hi]
//     flags: bit0=M2 bit1=M3 bit3=servo bit4=estop bit5=M1 bit6=M4
//   S3→FPGA: 16字节状态帧, 含四轴、两舵机、电磁铁和错误位
//
// I2C 寄存器 (龙芯→FPGA):
//   0x00=CMD, 0x05-06=speed
//   推荐32位脉冲: 0x40-43=M1, 0x44-47=M2, 0x48-4B=M3, 0x4C-4F=M4 (BE写入)
//   旧16位入口0x01-04/0x08-0B保留兼容并执行符号扩展
//   0x0C=EM推杆(直连引脚)  0x0D=servo1  0x0E=servo2
//   0x10-17=M2/M3位置(RO)  0x18-1B=M1位置电流(RO)  0x20-23=M4位置电流(RO)
//   0x28=status(RO)
//******************************************************************************

module top (
    input  wire clk_50m, input  wire rst_n,
    inout  wire i2c_scl,  inout  wire i2c_sda,
    output wire router_txd, input wire router_rxd, // → 底盘 S3 路由器
    output wire dbg_txd,                           // 调试 TX only
    output wire beep
);

    localparam CLK_FREQ = 50_000_000;
    localparam UART_BPS = 115200;

    function [7:0] crc8_byte;
        input [7:0] crc_in; input [7:0] data_in;
        integer k; reg [7:0] c;
        begin c=crc_in^data_in; for(k=0;k<8;k=k+1) c=c[7]?((c<<1)^8'h07):(c<<1); crc8_byte=c; end
    endfunction
    function [7:0] crc8_frame12;
        input [95:0] data; integer k; reg [7:0] c;
        begin c=0; for(k=0;k<12;k=k+1)c=crc8_byte(c,data[k*8 +: 8]); crc8_frame12=c; end
    endfunction
    function [7:0] crc8_frame17;
        input [135:0] data; integer k; reg [7:0] c;
        begin c=0; for(k=0;k<17;k=k+1)c=crc8_byte(c,data[k*8 +: 8]); crc8_frame17=c; end
    endfunction

    //==========================================================================
    // I2C Slave
    //==========================================================================
    wire i2c_rx_valid; wire [7:0] i2c_rx_addr, i2c_rx_data;
    wire i2c_tx_req;   wire [7:0] i2c_tx_addr;
    reg  [7:0] i2c_tx_data;

    i2c_slave #(.SLAVE_ADDR(7'h20)) u_i2c (
        .clk(clk_50m), .rst_n(rst_n),
        .scl(i2c_scl), .sda(i2c_sda),
        .rx_valid(i2c_rx_valid), .rx_addr(i2c_rx_addr), .rx_data(i2c_rx_data),
        .tx_req(i2c_tx_req), .tx_addr(i2c_tx_addr), .tx_data(i2c_tx_data),
        .i2c_active()
    );

    //==========================================================================
    // UART → 底盘 S3 路由器
    //==========================================================================
    wire s_rx_done, s_tx_ready; wire [7:0] s_rx_data;
    reg tx_start; reg [7:0] tx_byte;
    reg dbg_start; reg [7:0] dbg_byte;
    uart_rx #(.CLK_FREQ(CLK_FREQ),.UART_BPS(UART_BPS))
        u_rx (.clk(clk_50m),.rst_n(rst_n),.rxd(router_rxd),
              .rx_done(s_rx_done),.rx_data(s_rx_data),.rx_idle(),.rx_error());
    uart_tx #(.CLK_FREQ(CLK_FREQ),.UART_BPS(UART_BPS))
        u_tx (.clk(clk_50m),.rst_n(rst_n),
              .tx_start(tx_start),.tx_data(tx_byte),.tx_ready(s_tx_ready),.txd(router_txd));

    // DBG UART
    wire dbg_tx_ready;
    uart_tx #(.CLK_FREQ(CLK_FREQ),.UART_BPS(UART_BPS))
        u_dbg (.clk(clk_50m),.rst_n(rst_n),
               .tx_start(dbg_start),.tx_data(dbg_byte),.tx_ready(dbg_tx_ready),.txd(dbg_txd));

    //==========================================================================
    // Register File
    //==========================================================================
    reg [7:0]  reg_cmd;
    reg [31:0] reg_m2_pulse, reg_m3_pulse, reg_m1_pulse, reg_m4_pulse;
    reg [15:0] reg_speed;
    reg [3:0]  em_target;
    reg [7:0]  em_duration, em_cooldown; // ×10ms
    reg [15:0] reg_sv1, reg_sv2;       // 舵机角度, 支持270°
    reg [15:0] reg_m2_pos, reg_m3_pos, reg_m1_pos, reg_m4_pos;
    reg [7:0]  reg_status;             // 底盘 S3 返回的 flags

    wire cmd_arm  = reg_cmd[7];        // 发送 M2+M3 帧
    wire cmd_base = reg_cmd[6];        // 发送 M1+M4 帧
    wire cmd_servo= reg_cmd[3];        // 发送舵机帧
    wire cmd_stop = reg_cmd[1];        // 急停
    wire cmd_em   = reg_cmd[0];        // 电磁铁命令
    wire invalid_speed = (reg_speed == 16'd0) || (reg_speed > 16'd1000);
    wire invalid_m1 = ($signed(reg_m1_pulse) < -32'sd12000) ||
                      ($signed(reg_m1_pulse) >  32'sd12000);
    wire invalid_m2 = ($signed(reg_m2_pulse) < -32'sd192000) ||
                      ($signed(reg_m2_pulse) >  32'sd192000);
    wire invalid_m3 = ($signed(reg_m3_pulse) < -32'sd10000) ||
                      ($signed(reg_m3_pulse) >  32'sd10000);
    wire invalid_servo = (reg_sv1 > 16'd270) || (reg_sv2 > 16'd180);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            reg_cmd <= 8'd0;
            reg_m2_pulse <= 32'd0; reg_m3_pulse <= 32'd0;
            reg_m1_pulse <= 32'd0; reg_m4_pulse <= 32'd0;
            reg_speed  <= 16'd300;
            reg_sv1    <= 8'd127;       // 默认中位
            reg_sv2    <= 8'd80;        // 默认开爪
            em_target  <= 4'd0; em_duration <= 8'd20; em_cooldown <= 8'd50; cycle_ctrl <= 1'b0;
        end else if (tx_done) begin
            reg_cmd <= 8'd0;  // 帧发送完成, 自动清零 CMD
        end else if (i2c_rx_valid) begin
            case (i2c_rx_addr)
                8'h00: reg_cmd          <= i2c_rx_data;
                // 旧版16位入口保留兼容；写入时执行有符号扩展
                8'h01: reg_m2_pulse[15:8] <= i2c_rx_data;
                8'h02: begin reg_m2_pulse[7:0] <= i2c_rx_data; reg_m2_pulse[31:16] <= {16{reg_m2_pulse[15]}}; end
                8'h03: reg_m3_pulse[15:8] <= i2c_rx_data;
                8'h04: begin reg_m3_pulse[7:0] <= i2c_rx_data; reg_m3_pulse[31:16] <= {16{reg_m3_pulse[15]}}; end
                8'h05: reg_speed[15:8]    <= i2c_rx_data;
                8'h06: reg_speed[7:0]     <= i2c_rx_data;
                8'h08: reg_m1_pulse[15:8] <= i2c_rx_data;
                8'h09: begin reg_m1_pulse[7:0] <= i2c_rx_data; reg_m1_pulse[31:16] <= {16{reg_m1_pulse[15]}}; end
                8'h0A: reg_m4_pulse[15:8] <= i2c_rx_data;
                8'h0B: begin reg_m4_pulse[7:0] <= i2c_rx_data; reg_m4_pulse[31:16] <= {16{reg_m4_pulse[15]}}; end
                8'h0C: em_target          <= i2c_rx_data[3:0];
                8'h2A: em_duration        <= i2c_rx_data;
                8'h2B: em_cooldown        <= i2c_rx_data;
                8'h0F: cycle_ctrl         <= i2c_rx_data[0];  // bit0: 1=忙 0=就绪
                8'h0D: reg_sv1[7:0]       <= i2c_rx_data;
                8'h0E: reg_sv2[7:0]       <= i2c_rx_data;
                8'h2C: reg_sv1[15:8]      <= i2c_rx_data;
                8'h2D: reg_sv2[15:8]      <= i2c_rx_data;
                // 推荐的32位有符号脉冲寄存器，均按大端顺序写入
                8'h40: reg_m1_pulse[31:24] <= i2c_rx_data;
                8'h41: reg_m1_pulse[23:16] <= i2c_rx_data;
                8'h42: reg_m1_pulse[15:8]  <= i2c_rx_data;
                8'h43: reg_m1_pulse[7:0]   <= i2c_rx_data;
                8'h44: reg_m2_pulse[31:24] <= i2c_rx_data;
                8'h45: reg_m2_pulse[23:16] <= i2c_rx_data;
                8'h46: reg_m2_pulse[15:8]  <= i2c_rx_data;
                8'h47: reg_m2_pulse[7:0]   <= i2c_rx_data;
                8'h48: reg_m3_pulse[31:24] <= i2c_rx_data;
                8'h49: reg_m3_pulse[23:16] <= i2c_rx_data;
                8'h4A: reg_m3_pulse[15:8]  <= i2c_rx_data;
                8'h4B: reg_m3_pulse[7:0]   <= i2c_rx_data;
                8'h4C: reg_m4_pulse[31:24] <= i2c_rx_data;
                8'h4D: reg_m4_pulse[23:16] <= i2c_rx_data;
                8'h4E: reg_m4_pulse[15:8]  <= i2c_rx_data;
                8'h4F: reg_m4_pulse[7:0]   <= i2c_rx_data;
                default: ;
            endcase
        end
    end

    // I2C 读
    always @(*) begin
        i2c_tx_data = 8'h00;
        case (i2c_tx_addr)
            8'h10: i2c_tx_data = reg_m2_pos[15:8];
            8'h11: i2c_tx_data = reg_m2_pos[7:0];
            8'h12: i2c_tx_data = reg_m3_pos[15:8];
            8'h13: i2c_tx_data = reg_m3_pos[7:0];
            8'h18: i2c_tx_data = reg_m1_pos[15:8];
            8'h19: i2c_tx_data = reg_m1_pos[7:0];
            8'h20: i2c_tx_data = reg_m4_pos[15:8];
            8'h21: i2c_tx_data = reg_m4_pos[7:0];
            8'h28: i2c_tx_data = reg_status;
            8'h30: i2c_tx_data = reg_sv1_pos;       // 舵机1 实际角度
            8'h31: i2c_tx_data = reg_sv2_pos;       // 舵机2 实际角度: [7]=busy, [2:0]=active_idx
            8'h32: i2c_tx_data = {4'd0, reg_em_state};
            8'h33: i2c_tx_data = reg_remote_error;
            8'h34: i2c_tx_data = reg_done_flags;
            8'h35: i2c_tx_data = reg_warn_flags;
            8'h36: i2c_tx_data = reg_reject_reason;
            8'h37: i2c_tx_data = reg_reject_count[15:8];
            8'h38: i2c_tx_data = reg_reject_count[7:0];
            default: i2c_tx_data = 8'h00;
        endcase
    end

// flags: bit0=unused bit1=M2 bit2=M3 bit3=servo bit4=estop bit5=M1 bit6=M4

    //==========================================================================
    // 底盘 S3 命令帧发送 FSM
    //
    // 触发: cmd_arm/cmd_base/cmd_servo/cmd_stop/cmd_em
    // 发送 13字节帧: AA flags value1(LE32) value2(LE32) speed(LE16) CRC8
    //==========================================================================
    localparam TX_IDLE = 3'd0, TX_BYTE = 3'd1, TX_DONE = 3'd2, TX_BASE = 3'd3;

    reg [2:0] tx_state; reg [3:0] tx_idx;
    reg [7:0] tx_flags;
    reg [31:0] tx_val1, tx_val2;
    reg [15:0] tx_spd;
    reg tx_done;
    reg tx_is_both;
    reg [7:0] reg_reject_reason;
    reg [15:0] reg_reject_count;
    wire [7:0] tx_crc = crc8_frame12({tx_spd[15:8],tx_spd[7:0],
        tx_val2[31:24],tx_val2[23:16],tx_val2[15:8],tx_val2[7:0],
        tx_val1[31:24],tx_val1[23:16],tx_val1[15:8],tx_val1[7:0],tx_flags,8'hAA});

    wire cmd_both = cmd_arm && cmd_base;  // ARM+BASE 同时触发

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE; tx_idx <= 4'd0;
            tx_start <= 1'b0; tx_byte <= 8'd0; tx_done <= 1'b0; tx_is_both <= 1'b0;
            tx_flags <= 8'd0; tx_val1 <= 32'd0; tx_val2 <= 32'd0; tx_spd <= 16'd0;
            reg_reject_reason <= 8'd0; reg_reject_count <= 16'd0;
        end else begin
            tx_start <= 1'b0; tx_done <= 1'b0;

            case (tx_state)

            TX_IDLE: begin
                if (cmd_stop) begin
                    tx_flags <= 8'h10; tx_val1 <= 32'd0; tx_val2 <= 32'd0; tx_spd <= 16'd0;
                    tx_idx <= 4'd0; tx_state <= TX_BYTE;
                end
                else if (cmd_em) begin
                    // 无传送带实机版：四路推杆固定缺席。兼容旧命令但不发往外设。
                    tx_done <= 1'b1; tx_state <= TX_DONE;
                end
                else if (cmd_servo) begin
                    if (invalid_servo) begin
                        reg_reject_reason <= 8'h04; reg_reject_count <= reg_reject_count + 16'd1;
                        tx_done <= 1'b1; tx_state <= TX_DONE;
                    end else begin
                        tx_flags <= 8'h08; tx_val1 <= {16'd0, reg_sv1}; tx_val2 <= {16'd0, reg_sv2};
                        tx_spd <= 16'd0; tx_idx <= 4'd0; tx_state <= TX_BYTE;
                    end
                end
                else if (cmd_both) begin
                    if (invalid_speed || invalid_m1 || invalid_m2 || invalid_m3) begin
                        reg_reject_reason <= 8'h01; reg_reject_count <= reg_reject_count + 16'd1;
                        tx_done <= 1'b1; tx_state <= TX_DONE;
                    end else begin
                        tx_flags <= 8'h06; tx_val1 <= reg_m2_pulse; tx_val2 <= reg_m3_pulse;
                        tx_spd <= reg_speed; tx_idx <= 4'd0; tx_state <= TX_BYTE;
                        tx_is_both <= 1'b1;
                    end
                end
                else if (cmd_arm) begin
                    if (invalid_speed || invalid_m2 || invalid_m3) begin
                        reg_reject_reason <= 8'h02; reg_reject_count <= reg_reject_count + 16'd1;
                        tx_done <= 1'b1; tx_state <= TX_DONE;
                    end else begin
                        tx_flags <= 8'h06; tx_val1 <= reg_m2_pulse; tx_val2 <= reg_m3_pulse;
                        tx_spd <= reg_speed; tx_idx <= 4'd0; tx_state <= TX_BYTE;
                    end
                end
                else if (cmd_base) begin
                    if (invalid_speed || invalid_m1) begin
                        reg_reject_reason <= 8'h03; reg_reject_count <= reg_reject_count + 16'd1;
                        tx_done <= 1'b1; tx_state <= TX_DONE;
                    end else begin
                        tx_flags <= 8'h20; tx_val1 <= reg_m1_pulse; tx_val2 <= 32'd0;
                        tx_spd <= reg_speed; tx_idx <= 4'd0; tx_state <= TX_BYTE;
                    end
                end
                else if (self_trig) begin
                    tx_flags <= self_flags; tx_val1 <= self_val1; tx_val2 <= self_val2;
                    tx_spd <= self_spd; tx_idx <= 4'd0; tx_state <= TX_BYTE;
                end
            end

            TX_BASE: begin   // cmd_both 的第二帧: BASE (仅M1)
                if (s_tx_ready && !tx_start) begin
                    case (tx_idx)
                        4'd0: tx_byte <= 8'hAA;
                        4'd1: tx_byte <= 8'h20;    // 仅M1
                        4'd2: tx_byte <= reg_m1_pulse[7:0];
                        4'd3: tx_byte <= reg_m1_pulse[15:8];
                        4'd4: tx_byte <= reg_m1_pulse[23:16];
                        4'd5: tx_byte <= reg_m1_pulse[31:24];
                        4'd6: tx_byte <= 8'd0;
                        4'd7: tx_byte <= 8'd0;
                        4'd8: tx_byte <= 8'd0;
                        4'd9: tx_byte <= 8'd0;
                        4'd10: tx_byte <= reg_speed[7:0];
                        4'd11: tx_byte <= reg_speed[15:8];
                        4'd12: tx_byte <= crc8_frame12({reg_speed[15:8],reg_speed[7:0],
                            32'd0,
                            reg_m1_pulse[31:24],reg_m1_pulse[23:16],reg_m1_pulse[15:8],reg_m1_pulse[7:0],8'h20,8'hAA});
                        default: tx_byte <= 8'h00;
                    endcase
                    tx_start <= 1'b1;
                    tx_idx <= tx_idx + 4'd1;
                    if (tx_idx == 4'd12) begin
                        tx_state <= TX_DONE;
                        tx_done <= 1'b1;
                    end
                end
            end

            TX_BYTE: begin
                if (s_tx_ready && !tx_start) begin
                    case (tx_idx)
                        4'd0: tx_byte <= 8'hAA;
                        4'd1: tx_byte <= tx_flags;
                        4'd2: tx_byte <= tx_val1[7:0];
                        4'd3: tx_byte <= tx_val1[15:8];
                        4'd4: tx_byte <= tx_val1[23:16];
                        4'd5: tx_byte <= tx_val1[31:24];
                        4'd6: tx_byte <= tx_val2[7:0];
                        4'd7: tx_byte <= tx_val2[15:8];
                        4'd8: tx_byte <= tx_val2[23:16];
                        4'd9: tx_byte <= tx_val2[31:24];
                        4'd10: tx_byte <= tx_spd[7:0];
                        4'd11: tx_byte <= tx_spd[15:8];
                        4'd12: tx_byte <= tx_crc;
                        default: tx_byte <= 8'h00;
                    endcase
                    tx_start <= 1'b1;
                    tx_idx <= tx_idx + 4'd1;
                    if (tx_idx == 4'd12) begin
                        tx_state <= TX_DONE;
                        tx_done <= 1'b1;
                    end
                end
            end

            TX_DONE: begin
                if (tx_is_both) begin
                    tx_is_both <= 1'b0;
                    tx_idx <= 4'd0;            // 重置索引
                    tx_state <= TX_BASE;       // 接着发第二帧 (BASE)
                end else if (!cmd_arm && !cmd_base && !cmd_servo && !cmd_stop && !cmd_em && !self_trig)
                    tx_state <= TX_IDLE;
            end

            default: tx_state <= TX_IDLE;
            endcase
        end
    end

    //==========================================================================
    // 上电自测: 3 秒后 → M2 下降 1cm → 停 5 秒 → M2 上升 1cm
    //   关闭自测: 设 SELF_TEST_EN = 0
    //==========================================================================
    localparam SELF_TEST_EN = 1'b0;  // 1=启用自测  0=跳过
    localparam TEST_IDLE  = 2'd0;
    localparam TEST_LOWER = 2'd1;
    localparam TEST_WAIT  = 2'd2;
    localparam TEST_RAISE = 2'd3;

    reg [1:0]  test_state; reg test_done_r;
    reg [27:0] test_timer;       // ~5.4s max at 50MHz
    reg        self_trig;
    reg [7:0]  self_flags;
    reg [31:0] self_val1, self_val2;
    reg [15:0] self_spd;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            test_state <= TEST_IDLE; test_timer <= 28'd0;
            self_trig  <= 1'b0;      test_done_r <= 1'b0;
            self_flags <= 8'd0; self_val1 <= 32'd0; self_val2 <= 32'd0; self_spd <= 16'd0;
        end else begin
            self_trig <= 1'b0;  // pulse only

            case (test_state)

            TEST_IDLE: begin
                if (test_done_r || !SELF_TEST_EN) ;  // 已完成或未启用, 不动
                else if (test_timer < 28'd150_000_000)
                    test_timer <= test_timer + 28'd1;
                else begin
                    // 触发: M2 下降 1cm (CCW, -16000)
                    self_flags <= 8'h02;   // bit1=M2 only
                    self_val1  <= -32'sd16000;
                    self_val2  <= 32'd0;
                    self_spd   <= 16'd500;
                    self_trig  <= 1'b1;
                    test_timer <= 28'd0;
                    test_state <= TEST_LOWER;
                end
            end

            TEST_LOWER: begin
                if (tx_done) begin
                    self_trig  <= 1'b0;
                    test_timer <= 28'd0;
                    test_state <= TEST_WAIT;
                end
            end

            TEST_WAIT: begin
                if (test_timer < 28'd250_000_000)     // 5 秒
                    test_timer <= test_timer + 28'd1;
                else begin
                    // 触发: M2 上升 1cm (CW, +16000)
                    self_flags <= 8'h02;
                    self_val1  <= 32'd16000;
                    self_val2  <= 32'd0;
                    self_spd   <= 16'd500;
                    self_trig  <= 1'b1;
                    test_timer <= 28'd0;
                    test_state <= TEST_RAISE;
                end
            end

            TEST_RAISE: begin
                if (tx_done) begin
                    self_trig  <= 1'b0;
                    test_done_r <= 1'b1;
                    test_state <= TEST_IDLE;  // 完成, 停在 IDLE
                end
            end

            default: test_state <= TEST_IDLE;
            endcase
        end
    end

    //==========================================================================
    // 底盘S3响应 — 18字节: 17字节数据 + CRC-8/0x07
    //==========================================================================
    reg [4:0]  rx_idx;        // 0-17
    reg [143:0] rx_buf;
    reg        rx_got;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            rx_idx <= 5'd0; rx_buf <= 144'd0; rx_got <= 1'b0;
        end else begin
            rx_got <= 1'b0;

            if (s_rx_done) begin
                if (rx_idx == 0 && s_rx_data == 8'hAA) begin
                    rx_idx <= 5'd1;
                    rx_buf[7:0] <= 8'hAA;
                end else if (rx_idx > 0 && rx_idx < 5'd18) begin
                    rx_buf[(rx_idx*8)+:8] <= s_rx_data;
                    rx_idx <= rx_idx + 5'd1;
                    if (rx_idx == 5'd17) begin
                        rx_got <= 1'b1;
                        rx_idx <= 5'd0;
                    end
                end else begin
                    rx_idx <= 5'd0;
                end
            end
        end
    end

    // 收到完整帧 → 更新寄存器
    reg [15:0] reg_sv1_pos, reg_sv2_pos;
    reg [3:0] reg_em_state;
    reg [7:0] reg_remote_error;
    reg [7:0] reg_warn_flags, reg_done_flags;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            reg_m2_pos <= 16'd0; reg_m3_pos <= 16'd0;
            reg_m1_pos <= 16'd0; reg_m4_pos <= 16'd0;
            reg_sv1_pos <= 8'd0; reg_sv2_pos <= 8'd0;
            reg_em_state <= 4'd0; reg_remote_error <= 8'd0;
            reg_warn_flags <= 8'h03; reg_done_flags <= 8'd0;
            reg_status <= 8'd0;
        end else if (rx_got && crc8_frame17(rx_buf[135:0]) == rx_buf[143:136]) begin
            reg_status  <= {rx_buf[15:9], busy};
            reg_m2_pos  <= {rx_buf[31:24], rx_buf[23:16]};
            reg_m3_pos  <= {rx_buf[47:40], rx_buf[39:32]};
            reg_m1_pos  <= {rx_buf[63:56], rx_buf[55:48]};
            reg_m4_pos  <= {rx_buf[79:72], rx_buf[71:64]};
            reg_sv1_pos <= {rx_buf[95:88],rx_buf[87:80]};
            reg_sv2_pos <= {rx_buf[111:104],rx_buf[103:96]};
            reg_em_state <= 4'd0;
            reg_warn_flags <= rx_buf[119:112];
            reg_remote_error <= rx_buf[127:120];
            reg_done_flags <= rx_buf[135:128];
        end
    end

    // BUSY: 龙芯软件控制的分拣周期忙标志
    //   EM 硬件激活 或 龙芯写入 cycle_ctrl=1 → BUSY=1
    //   龙芯写入 cycle_ctrl=0 → BUSY=0 (整个分拣周期结束, 可开始下一轮视觉)
    reg cycle_ctrl;  // 0x0F bit0, 龙芯写1=忙 写0=就绪
    wire busy = cycle_ctrl;

    //==========================================================================
    // Beep — 诊断: TX完成短鸣20ms / 收到S3帧短鸣
    //==========================================================================
    reg [23:0] beep_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) beep_cnt <= 24'd0;
        else if (tx_done) beep_cnt <= 24'd1_000_000;  // TX完成鸣20ms
        else if (beep_cnt > 0) beep_cnt <= beep_cnt - 24'd1;
    end
    assign beep = (beep_cnt > 0);

    //==========================================================================
    // Debug UART — 收到 S3 帧时打印各轴位置 (hex)
    //==========================================================================
    reg [2:0] dbg_state; reg [3:0] dbg_idx;
    reg [95:0] dbg_snap; reg dbg_trig;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            dbg_state <= 3'd0; dbg_idx <= 4'd0;
            dbg_start <= 1'b0; dbg_byte <= 8'd0;
            dbg_snap <= 96'd0; dbg_trig <= 1'b0;
        end else begin
            dbg_start <= 1'b0;

            if (rx_got) begin
                dbg_snap <= rx_buf; dbg_trig <= 1'b1;
                dbg_state <= 3'd1; dbg_idx <= 4'd0;
            end

            if (dbg_trig && dbg_state == 3'd1) begin
                if (dbg_tx_ready && !dbg_start) begin
                    case (dbg_idx)
                        4'd0:  dbg_byte <= 8'hBB;
                        4'd1:  dbg_byte <= dbg_snap[15:8];
                        4'd2:  dbg_byte <= dbg_snap[23:16];
                        4'd3:  dbg_byte <= dbg_snap[31:24];
                        4'd4:  dbg_byte <= dbg_snap[39:32];
                        4'd5:  dbg_byte <= dbg_snap[47:40];
                        4'd6:  dbg_byte <= dbg_snap[55:48];
                        4'd7:  dbg_byte <= dbg_snap[63:56];
                        4'd8:  dbg_byte <= dbg_snap[71:64];
                        4'd9:  dbg_byte <= dbg_snap[79:72];
                        4'd10: dbg_byte <= dbg_snap[87:80];   // SV1
                        4'd11: dbg_byte <= dbg_snap[95:88];   // SV2
                        default: dbg_byte <= 8'h00;
                    endcase
                    dbg_start <= 1'b1;
                    dbg_idx <= dbg_idx + 4'd1;
                    if (dbg_idx == 4'd11) begin
                        dbg_trig <= 1'b0; dbg_state <= 3'd0;
                    end
                end
            end
        end
    end

endmodule
