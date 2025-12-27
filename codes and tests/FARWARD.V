// forwarding_unit.v - Fixed with EXMEM SP Forwarding
// CRITICAL FIX: Forward SP from EXMEM stage (not MEMWB)

module forwarding_unit(
    input wire EXMEM_valid,
    input wire [3:0] EXMEM_opcode,
    input wire [1:0] EXMEM_ra,
    input wire [1:0] EXMEM_rb,
    input wire [7:0] EXMEM_result,
    input wire decr,
    input wire EXMEM_update_sp,      // NEW: SP update from MEM stage
    input wire [7:0] EXMEM_new_sp,   // NEW: New SP from MEM stage
    
    input wire MEMWB_valid,
    input wire MEMWB_write_enable,
    input wire [1:0] MEMWB_dest_reg,
    input wire [7:0] MEMWB_data,
    input wire MEMWB_update_sp,      // SP update from WB stage
    input wire [7:0] MEMWB_new_sp,   // New SP from WB stage
    
    output reg forward_ex_valid,
    output reg [1:0] forward_ex_reg,
    output reg [7:0] forward_ex_data,
    output reg forward_mem_valid,
    output reg [1:0] forward_mem_reg,
    output reg [7:0] forward_mem_data
);

    always @(*) begin
        forward_ex_valid = 1'b0;
        forward_mem_valid = 1'b0;
        forward_ex_reg = 2'b00;
        forward_mem_reg = 2'b00;
        forward_ex_data = 8'h00;
        forward_mem_data = 8'h00;
        
        // ========================================
        // Forwarding from EX stage (EXMEM registers)
        // ========================================
        if (EXMEM_valid) begin
            case (EXMEM_opcode)
                4'h1, 4'h2, 4'h3, 4'h4, 4'h5: begin // MOV, ADD, SUB, AND, OR
                    forward_ex_valid = 1'b1;
                    forward_ex_reg = EXMEM_ra;
                    forward_ex_data = EXMEM_result;
                end
                4'h8: begin // NOT/NEG/INC/DEC
                    forward_ex_valid = 1'b1;
                    forward_ex_reg = EXMEM_rb;
                    forward_ex_data = EXMEM_result;
                end
                4'hC: begin // LDM
                    if (EXMEM_ra == 2'b00) begin
                        forward_ex_valid = 1'b1;
                        forward_ex_reg = EXMEM_rb;
                        forward_ex_data = EXMEM_result;
                    end
                end
                4'h7: begin // IN
                    if (EXMEM_ra == 2'b11) begin
                        forward_ex_valid = 1'b1;
                        forward_ex_reg = EXMEM_rb;
                        forward_ex_data = EXMEM_result;
                    end
                end
                4'h6: begin // RLC, RRC
                    if (EXMEM_ra <= 2'b01) begin
                        forward_ex_valid = 1'b1;
                        forward_ex_reg = EXMEM_rb;
                        forward_ex_data = EXMEM_result;
                    end
                end
                4'hA: begin // LOOP
                    forward_ex_valid = 1'b1;
                    forward_ex_reg = EXMEM_ra;
                    forward_ex_data = EXMEM_result;
                end
            endcase
            
            // CRITICAL FIX: Forward SP from EXMEM stage!
            // This is available during MEM stage, so decode can get it in time
            if (EXMEM_update_sp&&decr) begin
                forward_ex_valid = 1'b1;
                forward_ex_reg = 2'b11;  // SP is register 3
                forward_ex_data = EXMEM_new_sp;
            end
        end
        
        // ========================================
        // Forwarding from MEM stage (MEMWB registers)
        // ========================================
        if (MEMWB_valid) begin
            // Regular register write forwarding
            if (MEMWB_write_enable) begin
                forward_mem_valid = 1'b1;
                forward_mem_reg = MEMWB_dest_reg;
                forward_mem_data = MEMWB_data;
            end
            
            // SP update forwarding from WB stage (backup)
            if (MEMWB_update_sp&&decr) begin
                forward_mem_valid = 1'b1;
                forward_mem_reg = 2'b11;
                forward_mem_data = MEMWB_new_sp;
            end
        end
    end

endmodule