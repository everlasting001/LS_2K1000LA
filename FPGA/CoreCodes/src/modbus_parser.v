//******************************************************************************
// modbus_parser — MODBUS Response Parser with CRC16 Verification
//
// Collects bytes from UART RX, detects frame boundaries via inter-byte gap
// (MODBUS RTU: 3.5 char times ≈ 304 μs @ 115200), computes CRC over entire
// frame, and presents parsed response fields.
//
// CRC check: computes CRC over ALL received bytes including the CRC bytes.
// On a valid frame, the final CRC value == 0x0000.
//
// Outputs:
//   resp_valid  : pulse when a valid frame has been received and CRC is OK
//   resp_addr   : MODBUS address
//   resp_func   : function code (bit7=1 indicates error response)
//   resp_dlen   : number of data bytes (function-specific)
//   resp_data   : up to 8 data bytes
//   is_error    : func & 0x80 (error response from device)
//******************************************************************************

module modbus_parser #(
    parameter CLK_FREQ = 50_000_000,
    parameter UART_BPS = 115200,
    parameter MAX_BYTES = 32
) (
    input  wire        clk,
    input  wire        rst_n,

    // === Byte input (from uart_rx) ===
    input  wire        rx_byte_valid,  // rx_done from uart_rx
    input  wire [7:0]  rx_byte,        // rx_data from uart_rx
    input  wire        rx_idle,        // rx_idle from uart_rx (>1ms quiet)

    // === Parsed response ===
    output reg         resp_valid,      // Pulse: valid response received
    output reg  [7:0]  resp_addr,
    output reg  [7:0]  resp_func,
    output reg  [7:0]  resp_dlen,       // Number of data bytes (0 for error)
    output reg  [63:0] resp_data,       // Up to 8 data bytes
    output reg         crc_ok,          // CRC verification passed
    output reg         is_error,        // Error response (func & 0x80)

    // === Status ===
    output wire        busy
);

    //==========================================================================
    // Frame gap detection
    // 3.5 char times @ 115200 = ~304 μs. At 50 MHz = 15200 cycles.
    // Use 16000 for margin (320 μs).
    //==========================================================================
    localparam GAP_LIMIT = 16_000;

    reg [13:0] gap_cnt;
    reg        frame_done;      // Gap detected → frame complete

    //==========================================================================
    // Byte buffer + CRC
    //==========================================================================
    reg [7:0]  byte_buf [0:MAX_BYTES-1];
    reg [4:0]  byte_cnt;      // Number of bytes in this frame (0..MAX_BYTES)

    // CRC interface
    reg         crc_init;
    reg         crc_valid;
    reg  [7:0]  crc_byte_r;     // Byte to feed to CRC (latched from rx)
    reg         crc_pending;     // A byte is waiting to be fed to CRC
    wire        crc_busy;
    wire        crc_done;
    wire [15:0] crc_out;

    // We feed each byte to CRC as it arrives, then feed the two CRC bytes
    // as well. Final CRC should be 0x0000 if valid.

    //==========================================================================
    // State machine
    //==========================================================================
    localparam ST_IDLE     = 2'd0;   // Waiting for first byte (line quiet)
    localparam ST_RECV     = 2'd1;   // Collecting bytes
    localparam ST_GAP      = 2'd2;   // Line went quiet, waiting for gap to confirm
    localparam ST_VALIDATE = 2'd3;   // Frame complete, checking CRC

    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            gap_cnt     <= 14'd0;
            frame_done  <= 1'b0;
            byte_cnt    <= 5'd0;
            crc_init    <= 1'b0;
            crc_pending <= 1'b0;
            resp_valid  <= 1'b0;
            resp_addr   <= 8'd0;
            resp_func   <= 8'd0;
            resp_dlen   <= 8'd0;
            resp_data   <= 64'd0;
            crc_ok      <= 1'b0;
            is_error    <= 1'b0;
        end else begin
            // Defaults
            crc_init   <= 1'b0;
            crc_valid  <= 1'b0;
            resp_valid <= 1'b0;

            case (state)

                //------------------------------------------------------------------
                ST_IDLE: begin
                    byte_cnt   <= 5'd0;
                    gap_cnt    <= 14'd0;
                    frame_done <= 1'b0;

                    if (rx_byte_valid) begin
                        // First byte of a new frame
                        crc_init     <= 1'b1;    // Reset CRC to 0xFFFF
                        byte_buf[0]       <= rx_byte;
                        byte_cnt     <= 5'd1;
                        // Feed first byte to CRC
                        crc_valid    <= 1'b1;
                        state        <= ST_RECV;
                    end
                end

                //------------------------------------------------------------------
                ST_RECV: begin
                    // Track gap: reset on new byte, increment otherwise
                    if (rx_byte_valid) begin
                        gap_cnt  <= 14'd0;
                        if (byte_cnt < MAX_BYTES) begin
                            byte_buf[byte_cnt] <= rx_byte;
                            byte_cnt      <= byte_cnt + 5'd1;
                        end
                        // Stage byte for CRC feeding
                        crc_byte_r  <= rx_byte;
                        crc_pending <= 1'b1;
                    end else begin
                        if (rx_idle && byte_cnt >= 4) begin
                            state <= ST_GAP;
                        end else if (gap_cnt < GAP_LIMIT) begin
                            gap_cnt <= gap_cnt + 14'd1;
                        end
                    end

                    // Feed pending byte to CRC when CRC is ready
                    if (crc_pending && !crc_busy) begin
                        crc_valid   <= 1'b1;
                        crc_pending <= 1'b0;
                    end
                end

                //------------------------------------------------------------------
                ST_GAP: begin
                    if (rx_byte_valid) begin
                        // More bytes arriving — frame continues
                        gap_cnt <= 14'd0;
                        if (byte_cnt < MAX_BYTES) begin
                            byte_buf[byte_cnt] <= rx_byte;
                            byte_cnt      <= byte_cnt + 5'd1;
                        end
                        crc_byte_r  <= rx_byte;
                        crc_pending <= 1'b1;
                        state <= ST_RECV;
                    end else if (gap_cnt < GAP_LIMIT) begin
                        gap_cnt <= gap_cnt + 14'd1;
                    end else begin
                        // Gap confirmed — frame is complete
                        // Feed any remaining pending byte to CRC before validating
                        frame_done <= 1'b1;
                        state      <= ST_VALIDATE;
                    end

                    // Feed pending byte to CRC when CRC is ready
                    if (crc_pending && !crc_busy) begin
                        crc_valid   <= 1'b1;
                        crc_pending <= 1'b0;
                    end
                end

                //------------------------------------------------------------------
                ST_VALIDATE: begin
                    // Feed any remaining pending byte first
                    if (crc_pending && !crc_busy) begin
                        crc_valid   <= 1'b1;
                        crc_pending <= 1'b0;
                    end

                    // Wait until CRC pipeline is fully drained
                    if (!crc_pending && !crc_busy) begin
                        // crc_out should be 0x0000 for a valid frame
                        crc_ok     <= (crc_out == 16'h0000);
                        resp_addr  <= byte_buf[0];
                        resp_func  <= byte_buf[1];
                        is_error   <= byte_buf[1][7];

                        if (byte_buf[1][7]) begin
                            resp_dlen  <= 8'd1;
                            resp_data  <= {56'd0, byte_buf[2]};
                        end else begin
                            resp_dlen  <= byte_buf[2];
                            resp_data  <= {
                                byte_buf[ 3], byte_buf[ 4], byte_buf[ 5], byte_buf[ 6],
                                byte_buf[ 7], byte_buf[ 8], byte_buf[ 9], byte_buf[10]
                            };
                        end

                        resp_valid <= 1'b1;
                        state      <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

    //==========================================================================
    // CRC16 instance — computes CRC over ALL received bytes
    //
    // NOTE: Current implementation feeds bytes to CRC as they arrive.
    // CRC16 takes 8 cycles/byte. At 115200 bps, bytes arrive every ~8680 cycles
    // (10 bits × 434 cycles/bit), so there's plenty of time between bytes.
    // The `!crc_busy` check in ST_RECV ensures we don't double-feed.
    //==========================================================================
    crc16 u_crc (
        .clk       (clk),
        .rst_n     (rst_n),
        .crc_init  (crc_init),
        .crc_byte  (crc_byte_r),
        .crc_valid (crc_valid && !crc_busy),
        .crc_busy  (crc_busy),
        .crc_done  (crc_done),
        .crc_out   (crc_out)
    );

    //==========================================================================
    // Status
    //==========================================================================
    assign busy = (state != ST_IDLE);

endmodule
