// IFID_reg.v - IF/ID Pipeline Register
// Fixed port names to match instantiation

module IFID_reg(
    input wire clk,
    input wire reset,
    input wire stall,
    input wire flush,
    input wire branch_taken,
    input wire [7:0] fetched_instruction,
    input wire [7:0] pc_in,
    
    output reg [7:0] IFID_instruction,
    output reg [7:0] IFID_pc,
    output reg IFID_valid
);

    always @(posedge clk) begin
        if (reset) begin
            IFID_instruction <= 8'h00;
            IFID_pc <= 8'h00;
            IFID_valid <= 1'b0;
        end
        else if (stall) begin
            // Keep current values during stall
        end
        else if (flush || branch_taken) begin
            IFID_instruction <= 8'h00;
            IFID_pc <= 8'h00;
            IFID_valid <= 1'b0;
        end
        else begin
            IFID_instruction <= fetched_instruction;
            IFID_pc <= pc_in;
            IFID_valid <= 1'b1;
        end
    end

endmodule