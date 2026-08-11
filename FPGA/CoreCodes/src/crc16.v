//******************************************************************************
// crc16 — MODBUS CRC16 Calculator
//
// Polynomial: 0xA001 (reversed 0x8005)
// Initial value: 0xFFFF
// Result appended LSB-first in MODBUS frames.
//
// Interface:
//   crc_init   : pulse to reset CRC to 0xFFFF
//   crc_byte   : input byte
//   crc_valid  : pulse to feed one byte into CRC calculation
//   crc_done   : pulse when calculation is complete (8 cycles after crc_valid)
//   crc_out    : 16-bit CRC result (valid when crc_done=1)
//
// Serial implementation: 8 cycles per byte, 1 LUT per bit.
// At 50MHz, one byte takes 160ns — negligible for 115200 bps UART.
//******************************************************************************

module crc16 (
    input  wire        clk,
    input  wire        rst_n,

    // Control
    input  wire        crc_init,      // Reset CRC to 0xFFFF
    input  wire [7:0]  crc_byte,      // Input data byte
    input  wire        crc_valid,     // Pulse: feed this byte

    // Output
    output wire        crc_busy,      // High while calculating (8 cycles)
    output wire        crc_done,      // Pulse when byte is processed
    output wire [15:0] crc_out        // Current CRC value
);

    reg [15:0] crc;
    reg [2:0]  bit_idx;
    reg        busy;

    assign crc_out  = crc;
    assign crc_busy = busy;
    assign crc_done = busy && (bit_idx == 3'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc     <= 16'hFFFF;
            bit_idx <= 3'd0;
            busy    <= 1'b0;
        end else begin
            if (crc_init) begin
                crc     <= 16'hFFFF;
                bit_idx <= 3'd0;
                busy    <= 1'b0;
            end else if (crc_valid && !busy) begin
                // XOR the new byte into the CRC LSB
                crc     <= crc ^ {8'd0, crc_byte};
                bit_idx <= 3'd7;        // Process 8 bits (7→0)
                busy    <= 1'b1;
            end else if (busy) begin
                if (bit_idx != 3'd0) begin
                    // Shift right and conditionally XOR polynomial
                    if (crc[0])
                        crc <= {1'b0, crc[15:1]} ^ 16'hA001;
                    else
                        crc <= {1'b0, crc[15:1]};
                    bit_idx <= bit_idx - 1;
                end else begin
                    busy <= 1'b0;
                end
            end
        end
    end

endmodule
