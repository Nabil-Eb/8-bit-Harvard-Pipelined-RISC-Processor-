// EXMEM_reg.v - EX/MEM Pipeline Register
// Extracted from working processor

module EXMEM_reg(
    input wire clk,
    input wire reset,
    input wire flush,
    
    input wire [7:0] result_in,
    input wire [7:0] operand_b_in,
    input wire [3:0] opcode_in,
    input wire [1:0] ra_in,
    input wire [1:0] rb_in,
    input wire [7:0] address_in,
    input wire valid_in,
    input wire [3:0] flags_in,
    
    output reg [7:0] EXMEM_result,
    output reg [7:0] EXMEM_operand_b,
    output reg [3:0] EXMEM_opcode,
    output reg [1:0] EXMEM_ra,
    output reg [1:0] EXMEM_rb,
    output reg [7:0] EXMEM_address,
    output reg EXMEM_valid,
    output reg [3:0] EXMEM_flags
);

    always @(posedge clk) begin
        if (reset || flush) begin
            EXMEM_result <= 8'h00;
            EXMEM_operand_b <= 8'h00;
            EXMEM_opcode <= 4'h0;
            EXMEM_ra <= 2'b00;
            EXMEM_rb <= 2'b00;
            EXMEM_address <= 8'h00;
            EXMEM_valid <= 1'b0;
            EXMEM_flags <= 4'h0;
        end
        else begin
            EXMEM_result <= result_in;
            EXMEM_operand_b <= operand_b_in;
            EXMEM_opcode <= opcode_in;
            EXMEM_ra <= ra_in;
            EXMEM_rb <= rb_in;
            EXMEM_address <= address_in;
            EXMEM_valid <= valid_in;
            EXMEM_flags <= flags_in;
        end
    end

endmodule