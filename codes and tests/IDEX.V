// IDEX_reg.v - ID/EX Pipeline Register
// Extracted from working processor

module IDEX_reg(
    input wire clk,
    input wire reset,
    input wire stall,
    input wire flush,
    input wire branch_taken,
    
    input wire [3:0] opcode_in,
    input wire [1:0] ra_in,
    input wire [1:0] rb_in,
    input wire [7:0] operand_a_in,
    input wire [7:0] operand_b_in,
    input wire [7:0] pc_in,
    input wire [7:0] imm_in,
    input wire is_two_byte_in,
    input wire valid_in,
    
    output reg [3:0] IDEX_opcode,
    output reg [1:0] IDEX_ra,
    output reg [1:0] IDEX_rb,
    output reg [7:0] IDEX_operand_a,
    output reg [7:0] IDEX_operand_b,
    output reg [7:0] IDEX_pc,
    output reg [7:0] IDEX_imm,
    output reg IDEX_is_two_byte,
    output reg IDEX_valid
);

    always @(posedge clk) begin
        if (reset || flush || branch_taken) begin
            IDEX_opcode <= 4'h0;
            IDEX_ra <= 2'b00;
            IDEX_rb <= 2'b00;
            IDEX_operand_a <= 8'h00;
            IDEX_operand_b <= 8'h00;
            IDEX_pc <= 8'h00;
            IDEX_imm <= 8'h00;
            IDEX_is_two_byte <= 1'b0;
            IDEX_valid <= 1'b0;
        end
        else if (stall) begin
            // Insert bubble (NOP)
            IDEX_opcode <= 4'h0;
            IDEX_ra <= 2'b00;
            IDEX_rb <= 2'b00;
            IDEX_operand_a <= 8'h00;
            IDEX_operand_b <= 8'h00;
            IDEX_valid <= 1'b0;
        end
        else begin
            IDEX_opcode <= opcode_in;
            IDEX_ra <= ra_in;
            IDEX_rb <= rb_in;
            IDEX_operand_a <= operand_a_in;
            IDEX_operand_b <= operand_b_in;
            IDEX_pc <= pc_in;
            IDEX_imm <= imm_in;
            IDEX_is_two_byte <= is_two_byte_in;
            IDEX_valid <= valid_in;
        end
    end

endmodule