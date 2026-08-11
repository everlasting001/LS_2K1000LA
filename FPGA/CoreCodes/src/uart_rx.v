//******************************************************************************
// uart_rx — UART Receiver with Frame Gap Detection
//
// Features:
//   - 3-stage synchronizer for asynchronous input
//   - Mid-bit sampling for noise immunity
//   - rx_done: one-cycle pulse when a complete byte is received
//   - rx_idle: asserted when line has been idle for >1ms (MODBUS frame gap)
//   - 1 start + 8 data + 1 stop, 115200 bps
//
// Interface:
//   rxd        : serial input
//   rx_done    : one-cycle pulse, data valid on this cycle
//   rx_data    : 8-bit received data, valid when rx_done=1
//   rx_idle    : high when no reception for >1ms (use as MODBUS frame delimiter)
//   rx_error   : high on framing error (stop bit not 1)
//******************************************************************************

module uart_rx #(
    parameter CLK_FREQ = 50_000_000,   // System clock frequency (Hz)
    parameter UART_BPS = 115200        // Baud rate (bps)
) (
    input  wire       clk,
    input  wire       rst_n,

    // Serial input
    input  wire       rxd,

    // Receive data interface
    output reg        rx_done,
    output reg  [7:0] rx_data,

    // Frame gap detection (for MODBUS protocol)
    output wire       rx_idle,

    // Error flag
    output reg        rx_error
);

    localparam BAUD_MAX    = CLK_FREQ / UART_BPS;   // ≈434
    localparam IDLE_MAX    = CLK_FREQ / 1000;       // 1ms @ 50MHz = 50000
    localparam HALF_BAUD   = BAUD_MAX / 2;          // Mid-bit sample point

    //**************************************************************************
    // 3-stage synchronizer for asynchronous rxd
    //**************************************************************************
    reg rxd_d0, rxd_d1, rxd_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxd_d0 <= 1'b1;
            rxd_d1 <= 1'b1;
            rxd_d2 <= 1'b1;
        end else begin
            rxd_d0 <= rxd;
            rxd_d1 <= rxd_d0;
            rxd_d2 <= rxd_d1;
        end
    end

    //**************************************************************************
    // Start bit detection: falling edge when not already receiving
    //**************************************************************************
    wire start_det;
    reg  rx_busy;
    assign start_det = rxd_d2 && ~rxd_d1 && ~rx_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_busy <= 1'b0;
        else if (start_det)
            rx_busy <= 1'b1;
        else if (rx_busy && bit_cnt == 4'd9 && baud_cnt == HALF_BAUD - 1)
            rx_busy <= 1'b0;   // Stop bit sampled — reception done
    end

    //**************************************************************************
    // Baud rate counter (runs during reception)
    //**************************************************************************
    reg [15:0] baud_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            baud_cnt <= 16'd0;
        else if (rx_busy) begin
            if (baud_cnt < BAUD_MAX - 1)
                baud_cnt <= baud_cnt + 16'd1;
            else
                baud_cnt <= 16'd0;
        end else begin
            baud_cnt <= 16'd0;
        end
    end

    //**************************************************************************
    // Bit counter (0=start detect, 1-8=data, 9=stop)
    //**************************************************************************
    reg [3:0] bit_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bit_cnt <= 4'd0;
        else if (rx_busy) begin
            if (baud_cnt == BAUD_MAX - 1)
                bit_cnt <= bit_cnt + 4'd1;
        end else begin
            bit_cnt <= 4'd0;
        end
    end

    //**************************************************************************
    // Sample data bits at mid-point
    //**************************************************************************
    reg [7:0] rx_data_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_data_r <= 8'd0;
        end else if (rx_busy && baud_cnt == HALF_BAUD - 1) begin
            case (bit_cnt)
                4'd1 : rx_data_r[0] <= rxd_d2;
                4'd2 : rx_data_r[1] <= rxd_d2;
                4'd3 : rx_data_r[2] <= rxd_d2;
                4'd4 : rx_data_r[3] <= rxd_d2;
                4'd5 : rx_data_r[4] <= rxd_d2;
                4'd6 : rx_data_r[5] <= rxd_d2;
                4'd7 : rx_data_r[6] <= rxd_d2;
                4'd8 : rx_data_r[7] <= rxd_d2;
                default : ;
            endcase
        end
    end

    //**************************************************************************
    // Output: rx_done pulse and rx_data latch
    //**************************************************************************
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_done  <= 1'b0;
            rx_data  <= 8'd0;
            rx_error <= 1'b0;
        end else begin
            rx_done <= 1'b0;   // Default: deassert after one cycle

            if (rx_busy && bit_cnt == 4'd9 && baud_cnt == HALF_BAUD - 1) begin
                rx_done  <= 1'b1;
                rx_data  <= rx_data_r;
                rx_error <= ~rxd_d2;   // Error if stop bit is not high
            end
        end
    end

    //**************************************************************************
    // Frame gap / idle detection (>1ms no activity → MODBUS frame delimiter)
    //**************************************************************************
    reg [15:0] idle_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            idle_cnt <= 16'd0;
        else if (rx_done || start_det)
            idle_cnt <= 16'd0;
        else if (idle_cnt < IDLE_MAX)
            idle_cnt <= idle_cnt + 16'd1;
    end

    assign rx_idle = (idle_cnt == IDLE_MAX);

endmodule
