// pc.v - Program Counter Module
// Fixed port names to match instantiation

module pc(
    input wire clk,
    input wire reset,
    input wire stall,
    input wire branch_taken,
    input wire [7:0] branch_target,
    input wire [7:0] reset_vector,
    input wire [7:0] fetched_instruction,  // For 2-byte detection
    
    output reg [7:0] pc_out
);

    // Check if instruction is 2-byte
    wire is_two_byte = (fetched_instruction[7:4] == 4'hC && fetched_instruction[3:2] != 2'b11);

    always @(posedge clk) begin
        if (reset) begin
            pc_out <= reset_vector;
        end
        else if (stall) begin
            // PC stays the same during stall
        end
        else if (branch_taken) begin
            pc_out <= branch_target;
        end
        else begin
            // Increment PC based on instruction length
            if (is_two_byte) begin
                pc_out <= pc_out + 2;
            end
            else begin
                pc_out <= pc_out + 1;
            end
        end
    end

endmodule