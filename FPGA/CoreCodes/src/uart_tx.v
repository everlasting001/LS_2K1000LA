//******************************************************************************
// uart_tx — UART Transmitter with Flow Control
//
// Features:
//   - Configurable baud rate via parameter
//   - tx_ready flow control: indicates module can accept next byte
//   - 1 start bit + 8 data bits (LSB first) + 1 stop bit, no parity
//
// Interface:
//   tx_start  : pulse high for 1 cycle to begin transmission
//   tx_data   : 8-bit parallel data, latched on tx_start
//   tx_ready  : high = ready to accept next byte; low = transmission in progress
//   txd       : serial output (idle high)
//******************************************************************************

module uart_tx #(
    parameter CLK_FREQ = 50_000_000,   // System clock frequency (Hz)
    parameter UART_BPS = 115200        // Baud rate (bps)
) (
    input  wire       clk,
    input  wire       rst_n,

    // Transmit command interface
    input  wire       tx_start,        // Pulse: start transmission
    input  wire [7:0] tx_data,         // Data byte to transmit

    // Flow control
    output wire       tx_ready,        // High = ready for next byte

    // Serial output
    output wire       txd
);

    // Baud rate divisor
    localparam BAUD_MAX = CLK_FREQ / UART_BPS;   // 50MHz/115200 ≈ 434

    // Registers
    reg  [7:0]  tx_data_r;
    reg         tx_busy;
    reg  [3:0]  bit_cnt;
    reg  [15:0] baud_cnt;

    //**************************************************************************
    // tx_ready: inverse of busy
    //**************************************************************************
    assign tx_ready = ~tx_busy;

    //**************************************************************************
    // Latch data and set busy on tx_start
    //   - No !tx_busy guard: always accept new data.
    //     If tx_start fires while still sending, we restart transmission
    //     (this prevents the 1-cycle race where rx_done arrives just as
    //      tx_busy is being cleared — the byte would be lost otherwise).
    //   - For normal use, check tx_ready before calling tx_start.
    //**************************************************************************
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_data_r <= 8'd0;
            tx_busy   <= 1'b0;
        end else if (tx_start) begin
            tx_data_r <= tx_data;
            tx_busy   <= 1'b1;
        end else if (tx_busy && bit_cnt == 4'd9 && baud_cnt == BAUD_MAX - 1) begin
            // Stop bit finished — transmission complete
            tx_data_r <= 8'd0;
            tx_busy   <= 1'b0;
        end
    end

    //**************************************************************************
    // Baud rate counter
    //**************************************************************************
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt <= 16'd0;
        end else if (tx_start) begin
            baud_cnt <= 16'd0;    // Reset on any tx_start (including overwrite)
        end else if (tx_busy) begin
            if (baud_cnt < BAUD_MAX - 1)
                baud_cnt <= baud_cnt + 16'd1;
            else
                baud_cnt <= 16'd0;
        end else begin
            baud_cnt <= 16'd0;
        end
    end

    //**************************************************************************
    // Bit counter (0=start, 1-8=data, 9=stop)
    //**************************************************************************
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt <= 4'd0;
        end else if (tx_start) begin
            bit_cnt <= 4'd0;
        end else if (tx_busy) begin
            if (baud_cnt == BAUD_MAX - 1)
                bit_cnt <= bit_cnt + 4'd1;
        end else begin
            bit_cnt <= 4'd0;
        end
    end

    //**************************************************************************
    // Serial output mux
    //**************************************************************************
    reg txd_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txd_r <= 1'b1;   // Idle high
        end else if (tx_busy) begin
            case (bit_cnt)
                4'd0 : txd_r <= 1'b0;               // Start bit
                4'd1 : txd_r <= tx_data_r[0];        // LSB
                4'd2 : txd_r <= tx_data_r[1];
                4'd3 : txd_r <= tx_data_r[2];
                4'd4 : txd_r <= tx_data_r[3];
                4'd5 : txd_r <= tx_data_r[4];
                4'd6 : txd_r <= tx_data_r[5];
                4'd7 : txd_r <= tx_data_r[6];
                4'd8 : txd_r <= tx_data_r[7];        // MSB
                4'd9 : txd_r <= 1'b1;                // Stop bit
                default : txd_r <= 1'b1;
            endcase
        end else begin
            txd_r <= 1'b1;
        end
    end

    assign txd = txd_r;

endmodule
