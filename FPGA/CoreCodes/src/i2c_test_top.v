//******************************************************************************
// i2c_test_top — 使用已验证的 i2c_slave (从 My_First_FPGA_Project)
//
// I2C 从机地址 0x20
// 寄存器:
//   0x00: R/W scratch
//   0x01: R   version (0x42)
//   0x02: R/W LED (bit0=beep)
//   0x10: R   counter[7:0]
//   0x11: R   counter[15:8]
//
// 测试: i2cdetect -y 1 → 0x20
//       i2cget -y 1 0x20 0x01 → 0x42
//******************************************************************************

module i2c_test_top (
    input  wire clk_50m,
    input  wire rst_n,

    // I2C — inout 直连, 和已验证模块一致
    inout  wire i2c_scl,
    inout  wire i2c_sda,

    // Debug UART (未用, 占住端口防止综合报错)
    output wire dbg_txd,
    input  wire dbg_rxd,

    output wire beep
);

    //==========================================================================
    // i2c_slave (已验证)
    //==========================================================================
    wire        rx_valid;
    wire [7:0]  rx_addr;
    wire [7:0]  rx_data;
    wire        tx_req;
    wire [7:0]  tx_addr;
    wire [7:0]  tx_data;
    wire        i2c_active;

    i2c_slave #(.SLAVE_ADDR(7'h20))
    u_i2c (
        .clk(clk_50m), .rst_n(rst_n),
        .scl(i2c_scl), .sda(i2c_sda),
        .rx_valid(rx_valid), .rx_addr(rx_addr), .rx_data(rx_data),
        .tx_req(tx_req), .tx_addr(tx_addr), .tx_data(tx_data),
        .i2c_active(i2c_active)
    );

    //==========================================================================
    // 寄存器
    //==========================================================================
    reg [7:0] reg_scratch;   // 0x00
    reg [7:0] reg_led;      // 0x02

    // ~1kHz 计数器 (50M / 50000 = 1kHz) — 两次 i2cget 之间可见递增
    reg [15:0] cnt_div;
    reg [7:0]  counter;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin cnt_div <= 16'd0; counter <= 8'd0; end
        else if (cnt_div < 16'd50000) cnt_div <= cnt_div + 16'd1;
        else begin cnt_div <= 16'd0; counter <= counter + 8'd1; end
    end

    // 单寄存器模式: 所有地址统一读写 scratch
    // (i2c_slave 的 tx_addr 有 1 周期管道延迟, 多寄存器需额外逻辑)
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) reg_scratch <= 8'd0;
        else if (rx_valid) reg_scratch <= rx_data;
    end

    assign tx_data = counter;
    assign beep = 1'b0;

    // Debug UART 未用
    assign dbg_txd = 1'b1;

endmodule
