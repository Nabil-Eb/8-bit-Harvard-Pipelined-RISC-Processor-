// data_memory.v - Data Memory
// Extracted from working processor

module data_memory(
    input wire clk,
    input wire [7:0] read_addr,
    input wire write_enable,
    input wire [7:0] write_addr,
    input wire [7:0] write_data,
    
    output wire [7:0] read_data
);

    reg [7:0] memory [0:255];
    
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            memory[i] = 8'h00;
        end
    end
    
    // Asynchronous read
    assign read_data = memory[read_addr];
    
    // Synchronous write
    always @(posedge clk) begin
        if (write_enable) begin
            memory[write_addr] <= write_data;
        end
    end

endmodule