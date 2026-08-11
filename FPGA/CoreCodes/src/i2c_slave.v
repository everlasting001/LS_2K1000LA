//****************************************************************************************//
// Module:     i2c_slave.v
// Descriptions: I2C 从机模块
//               - 支持标准模式 (100kHz) 和快速模式 (400kHz)
//               - 7-bit 地址, 不支持 10-bit
//               - Slave 寄存器读写: Host 写地址+W → 写寄存器地址 → 写数据 (可多字节)
//               - Host 读: 发地址+R → 读数据 (可多字节)
//
// 使用时:
//   龙芯 I2C Master 写 FPGA:
//     START + SLAVE_ADDR(W) + REG_ADDR + DATA[0] + DATA[1] + ... + STOP
//   龙芯 I2C Master 读 FPGA:
//     START + SLAVE_ADDR(W) + REG_ADDR + RESTART + SLAVE_ADDR(R) + DATA[0] + ... + STOP
//
// 引脚: SCL=C22, SDA=D20 (I2C-1, LS2K_IIC1)
//****************************************************************************************//

module i2c_slave #(
    parameter SLAVE_ADDR = 7'h20       // I2C 从机地址 (默认 0x20)
) (
    input  wire         clk,            // 系统时钟 50MHz
    input  wire         rst_n,          // 复位, 低有效

    // I2C 总线
    inout  wire         scl,            // I2C 时钟 (C22)
    inout  wire         sda,            // I2C 数据 (D20)

    // 用户寄存器接口
    output wire         rx_valid,       // 收到完整寄存器写操作
    output wire [7:0]   rx_addr,        // 寄存器地址
    output wire [7:0]   rx_data,        // 收到的数据
    output wire         tx_req,         // 读请求 (需要用户提供数据)
    output wire [7:0]   tx_addr,        // 被读的寄存器地址
    input  wire [7:0]   tx_data,        // 用户返回的读数据

    // 状态
    output wire         i2c_active      // I2C 总线活动指示
);

    //************************************************************************************//
    //  输入同步 (SCL, SDA 异步于 clk)
    //************************************************************************************//

    reg [2:0] scl_sync;
    reg [2:0] sda_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 3'b111;
            sda_sync <= 3'b111;
        end else begin
            scl_sync <= {scl_sync[1:0], scl};
            sda_sync <= {sda_sync[1:0], sda};
        end
    end

    wire scl_rise = (scl_sync[2:1] == 2'b01);   // SCL 上升沿
    wire scl_fall = (scl_sync[2:1] == 2'b10);   // SCL 下降沿
    wire sda_rise = (sda_sync[2:1] == 2'b01);   // SDA 上升沿
    wire sda_fall = (sda_sync[2:1] == 2'b10);   // SDA 下降沿
    wire scl_high = scl_sync[1];
    wire sda_high = sda_sync[1];

    // START: SDA 下降沿 且 SCL 高
    wire start_det = sda_fall && scl_high;
    // STOP:  SDA 上升沿 且 SCL 高
    wire stop_det  = sda_rise && scl_high;

    //************************************************************************************//
    //  SDA 输出控制 (开漏, FPGA 只能拉低, 靠外部上拉拉高)
    //************************************************************************************//

    reg  sda_out;         // 1=高阻(释放), 0=拉低
    assign sda = sda_out ? 1'bz : 1'b0;

    reg  sda_oe_shadow;   // 输出使能
    wire sda_in = sda_sync[1];

    //************************************************************************************//
    //  位计数器
    //************************************************************************************//

    reg  [3:0]  bit_cnt;        // 0-7: 数据位, 8: ACK
    reg  [7:0]  shift_reg;      // 移位寄存器
    reg         rw_bit;         // 1=读, 0=写
    reg  [6:0]  addr_match;     // 收到的地址

    //************************************************************************************//
    //  字节级状态机
    //************************************************************************************//

    localparam S_IDLE      = 4'd0;   // 空闲
    localparam S_ADDR      = 4'd1;   // 收地址
    localparam S_ACK_ADDR  = 4'd2;   // 地址 ACK
    localparam S_REG       = 4'd3;   // 收寄存器地址
    localparam S_ACK_REG   = 4'd4;   // 寄存器地址 ACK
    localparam S_DATA_WR   = 4'd5;   // 收数据(写)
    localparam S_ACK_WR    = 4'd6;   // 写数据 ACK
    localparam S_DATA_RD   = 4'd7;   // 发数据(读)
    localparam S_ACK_RD    = 4'd8;   // 等 Master ACK
    localparam S_WAIT      = 4'd9;   // 等待

    reg  [3:0]  state;
    reg  [3:0]  next_state;
    reg  [7:0]  reg_addr;       // 当前寄存器地址
    reg         data_phase;     // 是否已收到寄存器地址
    reg         is_read;        // 当前是读操作
    reg  [7:0]  rd_shift;       // 读移位寄存器

    // 输出寄存器
    reg         rx_valid_reg;
    reg  [7:0]  rx_addr_reg;
    reg  [7:0]  rx_data_reg;
    reg         tx_req_reg;
    reg  [7:0]  tx_addr_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            next_state  <= S_IDLE;
            bit_cnt     <= 4'd0;
            shift_reg   <= 8'd0;
            rw_bit      <= 1'b0;
            addr_match  <= 7'd0;
            data_phase  <= 1'b0;
            is_read     <= 1'b0;
            reg_addr    <= 8'd0;
            rd_shift    <= 8'd0;
            sda_out     <= 1'b1;        // 释放总线
            sda_oe_shadow <= 1'b0;

            rx_valid_reg <= 1'b0;
            rx_addr_reg  <= 8'd0;
            rx_data_reg  <= 8'd0;
            tx_req_reg   <= 1'b0;
            tx_addr_reg  <= 8'd0;
        end else begin
            // 默认值
            rx_valid_reg <= 1'b0;
            tx_req_reg   <= 1'b0;

            // START / STOP 检测
            if (start_det) begin
                bit_cnt    <= 4'd0;
                shift_reg  <= 8'd0;
                data_phase <= 1'b0;
                state      <= S_ADDR;
                sda_out    <= 1'b1;
            end

            if (stop_det) begin
                state    <= S_IDLE;
                sda_out  <= 1'b1;
            end

            // SCL 边沿处理
            case (state)
                // --- 收地址字节 (7bit addr + R/W) ---
                S_ADDR: begin
                    if (scl_rise) begin
                        if (bit_cnt < 4'd8) begin
                            shift_reg <= {shift_reg[6:0], sda_in};
                            bit_cnt   <= bit_cnt + 1'b1;
                        end
                    end
                    if (scl_fall && bit_cnt == 4'd8) begin
                        // 检查地址
                        if (shift_reg[7:1] == SLAVE_ADDR) begin
                            rw_bit   <= shift_reg[0];
                            state    <= S_ACK_ADDR;
                            sda_out  <= 1'b0;      // ACK: 拉低 SDA
                        end else begin
                            state    <= S_IDLE;
                            sda_out  <= 1'b1;      // NACK: 释放
                        end
                    end
                end

                // --- 地址 ACK ---
                S_ACK_ADDR: begin
                    if (scl_fall) begin
                        bit_cnt <= 4'd0;
                        if (rw_bit) begin
                            // 读操作: 立即设第一个数据 bit (MSB), 不释放
                            rd_shift    <= tx_data;
                            sda_out     <= tx_data[7];   // MSB 现在就设!
                            state       <= S_DATA_RD;
                            tx_req_reg  <= 1'b1;
                            tx_addr_reg <= reg_addr;
                        end else begin
                            sda_out <= 1'b1;           // 写操作: 释放 ACK
                            // 写操作: 收下一字节
                            shift_reg <= 8'd0;
                            state     <= S_REG;
                        end
                    end
                end

                // --- 收寄存器地址 ---
                S_REG: begin
                    if (scl_rise && bit_cnt < 4'd8) begin
                        shift_reg <= {shift_reg[6:0], sda_in};
                        bit_cnt   <= bit_cnt + 1'b1;
                    end
                    if (scl_fall && bit_cnt == 4'd8) begin
                        reg_addr  <= shift_reg;
                        // SMBus read_byte_data uses W+register+RESTART+R.
                        // Publish the requested address here so tx_data has a
                        // full system-clock interval to settle before read ACK.
                        tx_addr_reg <= shift_reg;
                        state     <= S_ACK_REG;
                        sda_out   <= 1'b0;         // ACK
                    end
                end

                // --- 寄存器地址 ACK ---
                S_ACK_REG: begin
                    if (scl_fall) begin
                        sda_out <= 1'b1;
                        bit_cnt <= 4'd0;
                        shift_reg <= 8'd0;
                        data_phase <= 1'b1;
                        state    <= S_DATA_WR;
                    end
                end

                // --- 收写数据 ---
                S_DATA_WR: begin
                    if (scl_rise && bit_cnt < 4'd8) begin
                        shift_reg <= {shift_reg[6:0], sda_in};
                        bit_cnt   <= bit_cnt + 1'b1;
                    end
                    if (scl_fall && bit_cnt == 4'd8) begin
                        // 写完成, 输出给用户
                        rx_valid_reg <= 1'b1;
                        rx_addr_reg  <= reg_addr;
                        rx_data_reg  <= shift_reg;
                        reg_addr     <= reg_addr + 1'b1;  // 自动递增地址
                        state        <= S_ACK_WR;
                        sda_out      <= 1'b0;             // ACK
                    end
                end

                // --- 写 ACK ---
                S_ACK_WR: begin
                    if (scl_fall) begin
                        sda_out <= 1'b1;
                        bit_cnt <= 4'd0;
                        shift_reg <= 8'd0;
                        state   <= S_DATA_WR;     // 继续收下一字节
                    end
                end

                // --- 发读数据 (MSB 已在 S_ACK_ADDR 设好) ---
                S_DATA_RD: begin
                    if (scl_fall) begin
                        if (bit_cnt < 4'd7) begin
                            // 左移, 输出下一位
                            rd_shift <= {rd_shift[6:0], 1'b0};
                            sda_out  <= rd_shift[6];
                            bit_cnt  <= bit_cnt + 1'b1;
                        end else begin
                            // 8 位发完, 释放 SDA 等 Master ACK
                            sda_out <= 1'b1;
                            state   <= S_ACK_RD;
                        end
                    end
                end

                // --- 等 Master ACK/NACK ---
                S_ACK_RD: begin
                    if (scl_rise) begin
                        bit_cnt <= 4'd0;
                        if (!sda_in) begin
                            // Master ACK → 准备下一字节, 直接设 MSB
                            rd_shift   <= tx_data;
                            sda_out    <= tx_data[7];
                            state      <= S_DATA_RD;
                            tx_req_reg <= 1'b1;
                            tx_addr_reg <= reg_addr;
                            reg_addr   <= reg_addr + 1'b1;
                        end else begin
                            // Master NACK → 传输结束
                            state <= S_IDLE;
                        end
                    end
                end

                // --- 空闲: 等 START ---
                default: begin
                    sda_out <= 1'b1;
                end
            endcase
        end
    end

    assign rx_valid  = rx_valid_reg;
    assign rx_addr   = rx_addr_reg;
    assign rx_data   = rx_data_reg;
    assign tx_req    = tx_req_reg;
    assign tx_addr   = tx_addr_reg;
    assign i2c_active = (state != S_IDLE);

endmodule
