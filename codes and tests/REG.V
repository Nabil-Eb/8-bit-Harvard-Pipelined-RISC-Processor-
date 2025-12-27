// register_file.v - Register File
// Extracted from working processor code

module register_file(
    input wire clk,
    input wire reset,
    
    // Read ports
    input wire [1:0] read_addr1,
    input wire [1:0] read_addr2,
    output wire [7:0] read_data1,
    output wire [7:0] read_data2,
    
    // Write port
    input wire write_enable,
    input wire [1:0] write_addr,
    input wire [7:0] write_data,
    
    // SP update port
    input wire update_sp,
    input wire [7:0] new_sp
);

    reg [7:0] registers [0:3];
    
    integer i;
    initial begin
        for (i = 0; i < 4; i = i + 1) begin
            registers[i] = 8'h00;
        end
        registers[3] = 8'hFF; // SP starts at 0xFF
    end
    
    // Asynchronous read
    assign read_data1 = registers[read_addr1];
    assign read_data2 = registers[read_addr2];
    
    // Synchronous write
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 4; i = i + 1) begin
                registers[i] <= 8'h00;
            end
            registers[3] <= 8'hFF;
        end
        else begin
            // SP update has priority
            if (update_sp) begin
                registers[3] <= new_sp;
            end
            
            // Normal register write
            if (write_enable) begin
                if (write_addr != 2'b11) begin
                    registers[write_addr] <= write_data;
                end
                else if (!update_sp) begin
                    registers[3] <= write_data;
                end
            end
        end
    end

endmodule