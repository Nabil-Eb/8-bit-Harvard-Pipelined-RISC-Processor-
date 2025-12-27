// ccr.v - Condition Code Register
// Extracted from working processor

module ccr(
    input wire clk,
    input wire reset,
    input wire MEMWB_valid,
    input wire [3:0] MEMWB_flags,
    
    output reg [3:0] ccr_out
);

    always @(posedge clk) begin
        if (reset) begin
            ccr_out <= 4'h0;
        end
        else if (MEMWB_valid) begin
            ccr_out <= MEMWB_flags;
        end
    end

endmodule