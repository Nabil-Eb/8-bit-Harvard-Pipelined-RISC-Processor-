// instruction_memory.v - Instruction Memory
// Extracted from working processor code

module instruction_memory(
    input wire [7:0] addr,
    output wire [7:0] instruction
);

    reg [7:0] memory [0:255];
    
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            memory[i] = 8'h00;
        end
    end
    
    assign instruction = memory[addr];

endmodule