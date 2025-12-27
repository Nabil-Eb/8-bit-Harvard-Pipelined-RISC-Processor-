// control_unit.v - Control Unit
// Generates all control signals for the processor

module control_unit(
    input wire [3:0] opcode,
    input wire [1:0] ra,
    input wire [1:0] rb,
    
    // Control signals
    output reg reg_write,           // Register write enable
    output reg mem_read,            // Memory read
    output reg mem_write,           // Memory write
    output reg mem_to_reg,          // Write memory data to register
    output reg alu_src,             // ALU source: 0=reg, 1=imm
    output reg is_branch,           // Is conditional branch
    output reg is_jump,             // Is unconditional jump/call/ret
    output reg is_two_byte,         // Is 2-byte instruction
    output reg update_flags,        // Update condition flags
    output reg is_stack_op,         // Is stack operation (PUSH/POP/CALL/RET)
    output reg sp_increment,        // SP++ (POP/RET)
    output reg sp_decrement,        // SP-- (PUSH/CALL)
    output reg [3:0] alu_op         // ALU operation selector
);

    always @(*) begin
        // Default values
        reg_write = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_to_reg = 1'b0;
        alu_src = 1'b0;
        is_branch = 1'b0;
        is_jump = 1'b0;
        is_two_byte = 1'b0;
        update_flags = 1'b0;
        is_stack_op = 1'b0;
        sp_increment = 1'b0;
        sp_decrement = 1'b0;
        alu_op = 4'h0;
        
        case (opcode)
            4'h0: begin // NOP
                // All defaults
                alu_op = 4'h0;
            end
            
            4'h1: begin // MOV
                reg_write = 1'b1;
                alu_op = 4'h1;
            end
            
            4'h2: begin // ADD
                reg_write = 1'b1;
                update_flags = 1'b1;
                alu_op = 4'h2;
            end
            
            4'h3: begin // SUB
                reg_write = 1'b1;
                update_flags = 1'b1;
                alu_op = 4'h3;
            end
            
            4'h4: begin // AND
                reg_write = 1'b1;
                update_flags = 1'b1;
                alu_op = 4'h4;
            end
            
            4'h5: begin // OR
                reg_write = 1'b1;
                update_flags = 1'b1;
                alu_op = 4'h5;
            end
            
            4'h6: begin // RLC, RRC, SETC, CLRC
                if (ra <= 2'b01) begin // RLC, RRC
                    reg_write = 1'b1;
                end
                update_flags = 1'b1; // All update carry flag
                alu_op = 4'h6;
            end
            
            4'h7: begin // PUSH, POP, OUT, IN
                is_stack_op = 1'b1;
                case (ra)
                    2'b00: begin // PUSH
                        mem_write = 1'b1;
                        sp_decrement = 1'b1;
                    end
                    2'b01: begin // POP
                        mem_read = 1'b1;
                        reg_write = 1'b1;
                        mem_to_reg = 1'b1;
                        sp_increment = 1'b1;
                    end
                    2'b10: begin // OUT
                        // Handled separately in processor
                    end
                    2'b11: begin // IN
                        reg_write = 1'b1;
                    end
                endcase
                alu_op = 4'h7;
            end
            
            4'h8: begin // NOT, NEG, INC, DEC
                reg_write = 1'b1;
                update_flags = 1'b1;
                alu_op = 4'h8;
            end
            
            4'h9: begin // JZ, JN, JC, JV
                is_branch = 1'b1;
                alu_op = 4'h9;
            end
            
            4'hA: begin // LOOP
                reg_write = 1'b1;
                is_branch = 1'b1;
                alu_op = 4'hA;
            end
            
            4'hB: begin // JMP, CALL, RET, RTI
                is_jump = 1'b1;
                case (ra)
                    2'b00: begin // JMP
                        // Just jump
                    end
                    2'b01: begin // CALL
                        mem_write = 1'b1;
                        sp_decrement = 1'b1;
                        is_stack_op = 1'b1;
                    end
                    2'b10, 2'b11: begin // RET, RTI
                        mem_read = 1'b1;
                        sp_increment = 1'b1;
                        is_stack_op = 1'b1;
                    end
                endcase
                alu_op = 4'hB;
            end
            
            4'hC: begin // LDM, LDD, STD
                is_two_byte = (ra != 2'b11);
                alu_src = 1'b1; // Use immediate
                case (ra)
                    2'b00: begin // LDM
                        reg_write = 1'b1;
                    end
                    2'b01: begin // LDD
                        mem_read = 1'b1;
                        reg_write = 1'b1;
                        mem_to_reg = 1'b1;
                    end
                    2'b10: begin // STD
                        mem_write = 1'b1;
                    end
                endcase
                alu_op = 4'hC;
            end
            
            4'hD: begin // LDI
                mem_read = 1'b1;
                reg_write = 1'b1;
                mem_to_reg = 1'b1;
                alu_op = 4'hD;
            end
            
            4'hE: begin // STI
                mem_write = 1'b1;
                alu_op = 4'hE;
            end
            
            default: begin
                // NOP behavior
                alu_op = 4'h0;
            end
        endcase
    end

endmodule