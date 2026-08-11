//******************************************************************************
// modbus_packer — MODBUS Frame Assembler (v3, wbuart32 pattern)
//
// Design principles:
//   1. crc_byte_r captures byte_in on EVERY cycle where byte_valid might fire.
//      This guarantees crc16 always sees the correct byte regardless of timing.
//   2. crc_pend flag tracks "byte waiting for CRC" — decoupled from byte_valid.
//   3. tx_pending tracks "byte waiting for UART TX" — same pattern.
//   4. CRC and TX run in parallel. CRC (8 cycles/byte) always beats TX (8680).
//   5. frame_end triggers a countdown: after all cmd bytes TX'd + CRC'd → tail.
//******************************************************************************

module modbus_packer #(
    parameter MAX_BYTES = 32
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  byte_in,
    input  wire        byte_valid,
    output wire        byte_ack,
    input  wire        frame_end,

    output wire [7:0]  byte_out,
    output wire        tx_valid,
    input  wire        tx_ready,

    output wire        busy,
    output wire        done
);

    //==========================================================================
    // State machine
    //==========================================================================
    localparam ST_IDLE      = 3'd0;   // Wait + CRC reset
    localparam ST_SEND_CMD  = 3'd1;   // Accept bytes, feed CRC, send TX
    localparam ST_WAIT_DONE = 3'd2;   // Last cmd byte TX'd, waiting CRC finish
    localparam ST_SEND_CRCL = 3'd3;   // Send CRC[7:0]
    localparam ST_SEND_CRCH = 3'd4;   // Send CRC[15:8]
    localparam ST_DONE      = 3'd5;

    reg [2:0] state;

    //==========================================================================
    // TX pipeline: 1-deep
    //==========================================================================
    reg        tx_pend;
    reg [7:0]  tx_byte;

    //==========================================================================
    // CRC pipeline: 1-deep
    //==========================================================================
    reg        crc_pend;          // Byte waiting for CRC16
    reg [7:0]  crc_byte_r;        // Registered byte for CRC16 (captured from byte_in)

    reg        crc_init;
    reg        crc_valid;
    wire       crc_busy;
    wire [15:0] crc_out;

    //==========================================================================
    // Frame tracking
    //==========================================================================
    reg        frame_seen;        // frame_end was asserted
    reg [7:0]  cmd_tx_count;      // Number of command bytes sent to TX
    reg [7:0]  cmd_total;         // Total command bytes (latched after frame_seen)

    //==========================================================================
    // Outputs
    //==========================================================================
    assign byte_out  = tx_byte;
    assign tx_valid  = tx_pend && tx_ready;

    // byte_ack: ready for next byte when in SEND_CMD state, TX pipeline empty,
    // and haven't seen frame_end yet (can't accept more after frame_end)
    assign byte_ack  = (state == ST_SEND_CMD) && !tx_pend && !frame_seen
                       && !(byte_valid && frame_end);  // don't overlap frame_end

    //==========================================================================
    // Capture byte_in whenever byte_valid fires.
    // crc_byte_r holds the last byte sent by sequencer.
    // When crc_valid fires, crc16 sees the correct (captured) byte.
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            crc_byte_r <= 8'd0;
        else if (byte_valid)
            crc_byte_r <= byte_in;
    end

    //==========================================================================
    // Main FSM
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            tx_pend     <= 1'b0;
            tx_byte     <= 8'd0;
            crc_pend    <= 1'b0;
            crc_init    <= 1'b0;
            crc_valid   <= 1'b0;
            frame_seen  <= 1'b0;
            cmd_tx_count <= 8'd0;
            cmd_total   <= 8'd0;
        end else begin
            // Defaults
            crc_init  <= 1'b0;
            crc_valid <= 1'b0;

            case (state)

                //--------------------------------------------------------------
                // IDLE: reset CRC, wait for first byte
                //--------------------------------------------------------------
                ST_IDLE: begin
                    crc_init     <= 1'b1;          // Reset CRC → 0xFFFF
                    frame_seen   <= 1'b0;
                    cmd_tx_count <= 8'd0;
                    cmd_total    <= 8'd0;
                    tx_pend      <= 1'b0;
                    crc_pend     <= 1'b0;

                    if (byte_valid) begin
                        // crc_init=1 this cycle; next cycle CRC=0xFFFF and ready.
                        // Don't feed CRC yet — wait for ST_SEND_CMD first cycle.
                        tx_byte  <= byte_in;
                        tx_pend  <= 1'b1;
                        cmd_tx_count <= 8'd1;
                        // crc_byte_r captured in separate always block
                        state    <= ST_SEND_CMD;
                    end
                end

                //--------------------------------------------------------------
                // SEND_CMD: feed CRC + send TX + accept new bytes
                //
                // On entry from IDLE: first byte is in crc_byte_r (captured).
                // CRC was just reset (crc_init pulsed), now ready.
                // Feed the first byte immediately.
                //--------------------------------------------------------------
                ST_SEND_CMD: begin

                    // --- Feed pending byte to CRC ---
                    // First cycle from IDLE: crc_pend was 0, but we have first
                    // byte in crc_byte_r. Detect: cmd_tx_count==1 && !crc_pend
                    // and first byte hasn't been fed yet.
                    if (!crc_pend && cmd_tx_count == 8'd1 && !crc_busy
                        && !byte_valid) begin
                        // First byte was captured in crc_byte_r at IDLE→SEND
                        // but crc_byte_r is overwritten by the separate always
                        // block on byte_valid. If byte_valid is not active now,
                        // crc_byte_r still holds the first byte.
                        crc_pend  <= 1'b1;
                    end

                    // --- Accept new byte ---
                    if (byte_valid && byte_ack) begin
                        // crc_byte_r captured in separate always block
                        crc_pend <= 1'b1;           // Mark for CRC feed
                        tx_byte  <= byte_in;
                        tx_pend  <= 1'b1;
                        cmd_tx_count <= cmd_tx_count + 8'd1;
                        if (frame_end)
                            frame_seen <= 1'b1;
                    end

                    // --- Feed CRC ---
                    if (crc_pend && !crc_busy) begin
                        crc_valid <= 1'b1;
                        crc_pend  <= 1'b0;
                    end

                    // --- Send to TX ---
                    if (tx_pend && tx_ready) begin
                        tx_pend <= 1'b0;
                    end

                    // --- Transition: all cmd bytes TX'd and frame ended ---
                    if (frame_seen && !tx_pend
                        && !(byte_valid && byte_ack)) begin
                        state <= ST_WAIT_DONE;
                    end
                end

                //--------------------------------------------------------------
                // WAIT_DONE: drain CRC pipeline
                //--------------------------------------------------------------
                ST_WAIT_DONE: begin
                    // Feed any remaining pending CRC byte
                    if (crc_pend && !crc_busy) begin
                        crc_valid <= 1'b1;
                        crc_pend  <= 1'b0;
                    end

                    // All CRC done → send CRC tail
                    if (!crc_pend && !crc_busy) begin
                        state <= ST_SEND_CRCL;
                    end
                end

                //--------------------------------------------------------------
                // SEND_CRCL: send CRC[7:0]
                //--------------------------------------------------------------
                ST_SEND_CRCL: begin
                    if (!tx_pend) begin
                        tx_byte <= crc_out[7:0];
                        tx_pend <= 1'b1;
                    end else if (tx_ready) begin
                        tx_pend <= 1'b0;
                        state   <= ST_SEND_CRCH;
                    end
                end

                //--------------------------------------------------------------
                // SEND_CRCH: send CRC[15:8]
                //--------------------------------------------------------------
                ST_SEND_CRCH: begin
                    if (!tx_pend) begin
                        tx_byte <= crc_out[15:8];
                        tx_pend <= 1'b1;
                    end else if (tx_ready) begin
                        tx_pend <= 1'b0;
                        state   <= ST_DONE;
                    end
                end

                //--------------------------------------------------------------
                ST_DONE: begin
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

    //==========================================================================
    // CRC16 instance
    //==========================================================================
    crc16 u_crc (
        .clk       (clk),
        .rst_n     (rst_n),
        .crc_init  (crc_init),
        .crc_byte  (crc_byte_r),
        .crc_valid (crc_valid),
        .crc_busy  (crc_busy),
        .crc_done  (),
        .crc_out   (crc_out)
    );

    assign busy = (state != ST_IDLE) && (state != ST_DONE);
    assign done = (state == ST_DONE);

endmodule
