//******************************************************************************
// uart_test_top — FPGA↔C3↔S3 无线电机控制测试
//
// 每 500ms 通过 UART 发 6 字节 FpgaCmd 给 C3:
//   0xAA 0x01 0xFF 0x7F 0x00 0x00   (读 M2 位置)
//
// C3 返回 6 字节 FpgaRsp:
//   0xAA STAT POS_HI POS_LO CUR_HI CUR_LO
//
// DBG UART 打印收到的位置值 (HEX 显示)
//******************************************************************************

module uart_test_top (
    input  wire clk_50m,
    input  wire rst_n,
    input  wire m1_rxd,
    output wire m1_txd,
    output wire dbg_txd,
    input  wire dbg_rxd,
    output wire beep
);

    localparam CLK_FREQ = 50_000_000;
    localparam UART_BPS = 115200;

    //==========================================================================
    // M1 UART → C3
    //==========================================================================
    wire        m1_rx_done;
    wire [7:0]  m1_rx_data;
    wire        m1_tx_ready;

    uart_rx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(UART_BPS))
    u_m1_rx (.clk(clk_50m), .rst_n(rst_n), .rxd(m1_rxd),
        .rx_done(m1_rx_done), .rx_data(m1_rx_data),
        .rx_idle(), .rx_error());

    uart_tx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(UART_BPS))
    u_m1_tx (.clk(clk_50m), .rst_n(rst_n),
        .tx_start(m1_tx_start), .tx_data(m1_tx_data),
        .tx_ready(m1_tx_ready), .txd(m1_txd));

    //==========================================================================
    // FpgaCmd v2: 8 字节多设备协议
    // 读 M2+M3 位置: 0xAA 0x07 0xFF 0x7F 0xFF 0x7F 0x00 0x00
    //   [0]=AA [1]=0x07(FL_M2|FL_M3|FL_READ) [2:3]=NOP [4:5]=NOP [6:7]=speed
    //==========================================================================
    localparam CMD_LEN = 8;
    wire [7:0] cmd_frame [0:CMD_LEN-1];
    assign cmd_frame[0] = 8'hAA;   // magic
    assign cmd_frame[1] = 8'h07;   // FL_M2 | FL_M3 | FL_READ
    assign cmd_frame[2] = 8'hFF;   // VAL_NOP lo
    assign cmd_frame[3] = 8'h7F;   // VAL_NOP hi
    assign cmd_frame[4] = 8'hFF;   // VAL_NOP lo
    assign cmd_frame[5] = 8'h7F;   // VAL_NOP hi
    assign cmd_frame[6] = 8'h00;   // speed lo
    assign cmd_frame[7] = 8'h00;   // speed hi

    //==========================================================================
    // Sequencer: 每 500ms 发 6 字节帧
    //==========================================================================
    localparam T_500MS = 25_000_000;

    reg [24:0] timer;
    reg [3:0]  seq_state;
    reg [3:0]  byte_idx;

    reg        m1_tx_start;
    reg [7:0]  m1_tx_data;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            timer       <= 25'd0;
            seq_state   <= 4'd0;
            byte_idx    <= 4'd0;
            m1_tx_start <= 1'b0;
            m1_tx_data  <= 8'd0;
        end else begin
            m1_tx_start <= 1'b0;

            case (seq_state)
                4'd0: begin  // Wait
                    if (timer < T_500MS)
                        timer <= timer + 25'd1;
                    else begin
                        timer    <= 25'd0;
                        byte_idx <= 4'd0;
                        seq_state <= 4'd1;
                    end
                end

                4'd1: begin  // Send 6 bytes
                    if (m1_tx_ready && !m1_tx_start) begin
                        if (byte_idx < CMD_LEN) begin
                            m1_tx_data  <= cmd_frame[byte_idx];
                            m1_tx_start <= 1'b1;
                            byte_idx    <= byte_idx + 4'd1;
                        end else begin
                            seq_state <= 4'd0;
                        end
                    end
                end

                default: seq_state <= 4'd0;
            endcase
        end
    end

    //==========================================================================
    // Debug UART → 打印收到的 FpgaRsp 位置值 (HEX)
    //==========================================================================
    wire        dbg_tx_ready;
    reg         dbg_tx_start;
    reg  [7:0]  dbg_tx_data;

    uart_tx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(UART_BPS))
    u_dbg_tx (.clk(clk_50m), .rst_n(rst_n),
        .tx_start(dbg_tx_start), .tx_data(dbg_tx_data),
        .tx_ready(dbg_tx_ready), .txd(dbg_txd));

    // 收到 M1 RX 字节 → 原样转发到 DBG TX (嗅探器)
    reg         sniff_pending;
    reg  [7:0]  sniff_byte;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            dbg_tx_data   <= 8'd0;
            dbg_tx_start  <= 1'b0;
            sniff_pending <= 1'b0;
            sniff_byte    <= 8'd0;
        end else begin
            dbg_tx_start <= 1'b0;
            if (m1_rx_done) begin
                sniff_byte    <= m1_rx_data;
                sniff_pending <= 1'b1;
            end
            if (sniff_pending && dbg_tx_ready) begin
                dbg_tx_data   <= sniff_byte;
                dbg_tx_start  <= 1'b1;
                sniff_pending <= 1'b0;
            end
        end
    end

    assign beep = 1'b0;

endmodule
