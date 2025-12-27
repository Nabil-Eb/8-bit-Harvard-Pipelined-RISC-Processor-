// stack_pointer.v - Stack Pointer Management
// Handles SP updates for stack operations

module stack_pointer(
    input wire clk,
    input wire reset,
    input wire update_enable,
    input wire [7:0] new_sp,
    
    output reg [7:0] sp_value
);

    always @(posedge clk) begin
        if (reset) begin
            sp_value <= 8'hFF;
        end
        else if (update_enable) begin
            sp_value <= new_sp;
        end
    end

endmodule