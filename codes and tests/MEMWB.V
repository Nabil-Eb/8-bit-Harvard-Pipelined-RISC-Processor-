// MEMWB_reg.v - MEM/WB Pipeline Register
// Extracted from working processor

module MEMWB_reg(
    input wire clk,
    input wire reset,
    input wire flush,
    
    input wire [7:0] data_in,
    input wire [1:0] dest_reg_in,
    input wire write_enable_in,
    input wire valid_in,
    input wire [3:0] flags_in,
    input wire update_sp_in,
    input wire [7:0] new_sp_in,
    
    output reg [7:0] MEMWB_data,
    output reg [1:0] MEMWB_dest_reg,
    output reg MEMWB_write_enable,
    output reg MEMWB_valid,
    output reg [3:0] MEMWB_flags,
    output reg MEMWB_update_sp,
    output reg [7:0] MEMWB_new_sp
);

    always @(posedge clk) begin
        if (reset || flush) begin
            MEMWB_data <= 8'h00;
            MEMWB_dest_reg <= 2'b00;
            MEMWB_write_enable <= 1'b0;
            MEMWB_valid <= 1'b0;
            MEMWB_flags <= 4'h0;
            MEMWB_update_sp <= 1'b0;
            MEMWB_new_sp <= 8'h00;
        end
        else begin
            MEMWB_data <= data_in;
            MEMWB_dest_reg <= dest_reg_in;
            MEMWB_write_enable <= write_enable_in;
            MEMWB_valid <= valid_in;
            MEMWB_flags <= flags_in;
            MEMWB_update_sp <= update_sp_in;
            MEMWB_new_sp <= new_sp_in;
        end
    end

endmodule