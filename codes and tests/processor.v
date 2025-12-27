// processor_top.v - Top Level Processor (Updated with Control Unit)
// Integrates all modules including control unit

module processor_top(
    input wire clk,
    input wire reset,
    input wire interrupt,
    input wire [7:0] in_port,
    output reg [7:0] out_port
);

    // ==================== WIRES ====================
    
    // PC & Instruction Memory
    wire [7:0] pc_out;
    wire [7:0] fetched_instruction;
    
    // IF/ID
    wire [7:0] IFID_instruction;
    wire [7:0] IFID_pc;
    wire IFID_valid;
    
    // Control Unit signals
    wire ctrl_reg_write;
    wire ctrl_mem_read;
    wire ctrl_mem_write;
    wire ctrl_mem_to_reg;
    wire ctrl_alu_src;
    wire ctrl_is_branch;
    wire ctrl_is_jump;
    wire ctrl_is_two_byte;
    wire ctrl_update_flags;
    wire ctrl_is_stack_op;
    wire ctrl_sp_increment;
    wire ctrl_sp_decrement;
    wire [3:0] ctrl_alu_op;
    
    // Control
    wire stall;
    reg branch_taken;
    reg [7:0] branch_target;
    reg flush;
    
    // Forwarding
    wire forward_ex_valid;
    wire [1:0] forward_ex_reg;
    wire [7:0] forward_ex_data;
    wire forward_mem_valid;
    wire [1:0] forward_mem_reg;
    wire [7:0] forward_mem_data;
    
    // Register File
    wire [7:0] reg_read_data1, reg_read_data2;
    
    // ID/EX - now includes control signals
    wire [3:0] IDEX_opcode;
    wire [1:0] IDEX_ra, IDEX_rb;
    wire [7:0] IDEX_operand_a, IDEX_operand_b;
    wire [7:0] IDEX_pc, IDEX_imm;
    wire IDEX_is_two_byte, IDEX_valid;
    
    // ID/EX Control signals (propagated through pipeline)
    reg [3:0] IDEX_ctrl_alu_op;
    reg IDEX_ctrl_reg_write;
    reg IDEX_ctrl_mem_read;
    reg IDEX_ctrl_mem_write;
    reg IDEX_ctrl_mem_to_reg;
    reg IDEX_ctrl_is_branch;
    reg IDEX_ctrl_is_jump;
    reg IDEX_ctrl_sp_increment;
    reg IDEX_ctrl_sp_decrement;
    
    // EX/MEM
    wire [7:0] EXMEM_result, EXMEM_operand_b, EXMEM_address;
    wire [3:0] EXMEM_opcode, EXMEM_flags;
    wire [1:0] EXMEM_ra, EXMEM_rb;
    wire EXMEM_valid;
    
    // EX/MEM Control signals
    reg [3:0] EXMEM_ctrl_alu_op;
    reg EXMEM_ctrl_reg_write;
    reg EXMEM_ctrl_mem_read;
    reg EXMEM_ctrl_mem_write;
    reg EXMEM_ctrl_mem_to_reg;
    reg EXMEM_ctrl_sp_increment;
    reg EXMEM_ctrl_sp_decrement;
    
    // MEM/WB
    wire [7:0] MEMWB_data, MEMWB_new_sp;
    wire [1:0] MEMWB_dest_reg;
    wire [3:0] MEMWB_flags;
    wire MEMWB_write_enable, MEMWB_valid, MEMWB_update_sp;
    
    // CCR
    wire [3:0] ccr_out;
    
    // Data Memory
    wire [7:0] data_mem_read;
    reg data_mem_write_en;
    reg [7:0] data_mem_write_addr, data_mem_write_data;
    
    // Decode stage signals
    reg [3:0] decode_opcode;
    reg [1:0] decode_ra, decode_rb;
    reg [7:0] decode_operand_a, decode_operand_b, decode_imm;
    reg decode_is_two_byte, decode_valid;
    
    // Execute stage signals
    reg [7:0] execute_result, execute_address;
    reg [3:0] execute_flags;
    reg execute_valid;
    reg take_branch;
    
    // Memory stage signals
    reg [7:0] memory_data;
    reg [1:0] memory_dest_reg;
    reg memory_write_enable, memory_update_sp, is_decre;
    reg [7:0] memory_new_sp;
    reg memory_valid;
    
    // ALU wires
    wire [7:0] alu_result;
    wire [3:0] alu_flags;
    
    // Interrupt handling
    reg interrupt_pending, in_interrupt;
    reg [7:0] interrupt_return_pc;
    reg [3:0] interrupt_flags;
    wire [1:0] reg_read_addr1;
    assign reg_read_addr1 = ctrl_is_stack_op ? 2'b11 : IFID_instruction[3:2];
    
    // ==================== MODULE INSTANTIATIONS ====================
    
    // PC
    pc pc_inst(
        .clk(clk),
        .reset(reset),
        .stall(stall),
        .branch_taken(branch_taken),
        .branch_target(branch_target),
        .reset_vector(data_mem_read),
        .fetched_instruction(fetched_instruction),
        .pc_out(pc_out)
    );
    
    // Instruction Memory
    instruction_memory imem_inst(
        .addr(pc_out),
        .instruction(fetched_instruction)
    );
    
    // IF/ID Register
    IFID_reg ifid_inst(
        .clk(clk),
        .reset(reset),
        .stall(stall),
        .flush(flush),
        .branch_taken(branch_taken),
        .fetched_instruction(fetched_instruction),
        .pc_in(pc_out),
        .IFID_instruction(IFID_instruction),
        .IFID_pc(IFID_pc),
        .IFID_valid(IFID_valid)
    );
    
    // Control Unit (instantiated in Decode stage)
    control_unit ctrl_inst(
        .opcode(IFID_instruction[7:4]),
        .ra(IFID_instruction[3:2]),
        .rb(IFID_instruction[1:0]),
        .reg_write(ctrl_reg_write),
        .mem_read(ctrl_mem_read),
        .mem_write(ctrl_mem_write),
        .mem_to_reg(ctrl_mem_to_reg),
        .alu_src(ctrl_alu_src),
        .is_branch(ctrl_is_branch),
        .is_jump(ctrl_is_jump),
        .is_two_byte(ctrl_is_two_byte),
        .update_flags(ctrl_update_flags),
        .is_stack_op(ctrl_is_stack_op),
        .sp_increment(ctrl_sp_increment),
        .sp_decrement(ctrl_sp_decrement),
        .alu_op(ctrl_alu_op)
    );
    
    // Hazard Unit
    hazard_unit hazard_inst(
        .IFID_valid(IFID_valid),
        .IFID_instruction(IFID_instruction),
        .IDEX_valid(IDEX_valid),
        .IDEX_opcode(IDEX_opcode),
        .IDEX_ra(IDEX_ra),
        .IDEX_rb(IDEX_rb),
        .stall(stall)
    );
    
    // Forwarding Unit
    forwarding_unit forward_inst(
        .EXMEM_valid(EXMEM_valid),
        .EXMEM_opcode(EXMEM_opcode),
        .EXMEM_ra(EXMEM_ra),
        .EXMEM_rb(EXMEM_rb),
        .EXMEM_result(EXMEM_result),
        .EXMEM_update_sp(memory_update_sp),  // Direct from combinational memory stage
        .decr(is_decre),
        .EXMEM_new_sp(memory_new_sp),        // Direct from combinational memory stage
        .MEMWB_valid(MEMWB_valid),
        .MEMWB_write_enable(MEMWB_write_enable),
        .MEMWB_dest_reg(MEMWB_dest_reg),
        .MEMWB_data(MEMWB_data),
        .MEMWB_update_sp(MEMWB_update_sp),
        .MEMWB_new_sp(MEMWB_new_sp),
        .forward_ex_valid(forward_ex_valid),
        .forward_ex_reg(forward_ex_reg),
        .forward_ex_data(forward_ex_data),
        .forward_mem_valid(forward_mem_valid),
        .forward_mem_reg(forward_mem_reg),
        .forward_mem_data(forward_mem_data)
    );
    
    // Register File
    register_file regfile_inst(
        .clk(clk),
        .reset(reset),
        .read_addr1(reg_read_addr1),
        .read_addr2(IFID_instruction[1:0]),
        .read_data1(reg_read_data1),
        .read_data2(reg_read_data2),
        .write_enable(MEMWB_write_enable),
        .write_addr(MEMWB_dest_reg),
        .write_data(MEMWB_data),
        .update_sp(MEMWB_update_sp),
        .new_sp(MEMWB_new_sp)
    );
    
    // ID/EX Register
    IDEX_reg idex_inst(
        .clk(clk),
        .reset(reset),
        .stall(stall),
        .flush(flush),
        .branch_taken(branch_taken),
        .opcode_in(decode_opcode),
        .ra_in(decode_ra),
        .rb_in(decode_rb),
        .operand_a_in(decode_operand_a),
        .operand_b_in(decode_operand_b),
        .pc_in(IFID_pc),
        .imm_in(decode_imm),
        .is_two_byte_in(decode_is_two_byte),
        .valid_in(decode_valid),
        .IDEX_opcode(IDEX_opcode),
        .IDEX_ra(IDEX_ra),
        .IDEX_rb(IDEX_rb),
        .IDEX_operand_a(IDEX_operand_a),
        .IDEX_operand_b(IDEX_operand_b),
        .IDEX_pc(IDEX_pc),
        .IDEX_imm(IDEX_imm),
        .IDEX_is_two_byte(IDEX_is_two_byte),
        .IDEX_valid(IDEX_valid)
    );
    
    // Propagate control signals through ID/EX
    always @(posedge clk) begin
        if (reset || flush || branch_taken || stall) begin
            IDEX_ctrl_alu_op <= 4'h0;
            IDEX_ctrl_reg_write <= 1'b0;
            IDEX_ctrl_mem_read <= 1'b0;
            IDEX_ctrl_mem_write <= 1'b0;
            IDEX_ctrl_mem_to_reg <= 1'b0;
            IDEX_ctrl_is_branch <= 1'b0;
            IDEX_ctrl_is_jump <= 1'b0;
            IDEX_ctrl_sp_increment <= 1'b0;
            IDEX_ctrl_sp_decrement <= 1'b0;
        end
        else if (IFID_valid) begin
            IDEX_ctrl_alu_op <= ctrl_alu_op;
            IDEX_ctrl_reg_write <= ctrl_reg_write;
            IDEX_ctrl_mem_read <= ctrl_mem_read;
            IDEX_ctrl_mem_write <= ctrl_mem_write;
            IDEX_ctrl_mem_to_reg <= ctrl_mem_to_reg;
            IDEX_ctrl_is_branch <= ctrl_is_branch;
            IDEX_ctrl_is_jump <= ctrl_is_jump;
            IDEX_ctrl_sp_increment <= ctrl_sp_increment;
            IDEX_ctrl_sp_decrement <= ctrl_sp_decrement;
        end
    end
    
    // ALU
    alu alu_inst(
        .operand_a(IDEX_operand_a),
        .operand_b(IDEX_operand_b),
        .opcode(IDEX_opcode),
        .ra(IDEX_ra),
        .ccr_in(ccr_out),
        .result(alu_result),
        .flags_out(alu_flags)
    );
    
    // EX/MEM Register
    EXMEM_reg exmem_inst(
        .clk(clk),
        .reset(reset),
        .flush(flush),
        .result_in(execute_result),
        .operand_b_in(IDEX_operand_b),
        .opcode_in(IDEX_opcode),
        .ra_in(IDEX_ra),
        .rb_in(IDEX_rb),
        .address_in(execute_address),
        .valid_in(execute_valid),
        .flags_in(execute_flags),
        .EXMEM_result(EXMEM_result),
        .EXMEM_operand_b(EXMEM_operand_b),
        .EXMEM_opcode(EXMEM_opcode),
        .EXMEM_ra(EXMEM_ra),
        .EXMEM_rb(EXMEM_rb),
        .EXMEM_address(EXMEM_address),
        .EXMEM_valid(EXMEM_valid),
        .EXMEM_flags(EXMEM_flags)
    );
    
    // Propagate control signals through EX/MEM
    always @(posedge clk) begin
        if (reset || flush) begin
            EXMEM_ctrl_alu_op <= 4'h0;
            EXMEM_ctrl_reg_write <= 1'b0;
            EXMEM_ctrl_mem_read <= 1'b0;
            EXMEM_ctrl_mem_write <= 1'b0;
            EXMEM_ctrl_mem_to_reg <= 1'b0;
            EXMEM_ctrl_sp_increment <= 1'b0;
            EXMEM_ctrl_sp_decrement <= 1'b0;
        end
        else if (IDEX_valid) begin
            EXMEM_ctrl_alu_op <= IDEX_ctrl_alu_op;
            EXMEM_ctrl_reg_write <= IDEX_ctrl_reg_write;
            EXMEM_ctrl_mem_read <= IDEX_ctrl_mem_read;
            EXMEM_ctrl_mem_write <= IDEX_ctrl_mem_write;
            EXMEM_ctrl_mem_to_reg <= IDEX_ctrl_mem_to_reg;
            EXMEM_ctrl_sp_increment <= IDEX_ctrl_sp_increment;
            EXMEM_ctrl_sp_decrement <= IDEX_ctrl_sp_decrement;
        end
    end
    
    // Data Memory
    data_memory dmem_inst(
        .clk(clk),
        .read_addr(EXMEM_address),
        .write_enable(data_mem_write_en),
        .write_addr(data_mem_write_addr),
        .write_data(data_mem_write_data),
        .read_data(data_mem_read)
    );
    
    // MEM/WB Register
    MEMWB_reg memwb_inst(
        .clk(clk),
        .reset(reset),
        .flush(flush),
        .data_in(memory_data),
        .dest_reg_in(memory_dest_reg),
        .write_enable_in(memory_write_enable),
        .valid_in(memory_valid),
        .flags_in(EXMEM_flags),
        .update_sp_in(memory_update_sp),
        .new_sp_in(memory_new_sp),
        .MEMWB_data(MEMWB_data),
        .MEMWB_dest_reg(MEMWB_dest_reg),
        .MEMWB_write_enable(MEMWB_write_enable),
        .MEMWB_valid(MEMWB_valid),
        .MEMWB_flags(MEMWB_flags),
        .MEMWB_update_sp(MEMWB_update_sp),
        .MEMWB_new_sp(MEMWB_new_sp)
    );
    
    // CCR
    ccr ccr_inst(
        .clk(clk),
        .reset(reset),
        .MEMWB_valid(MEMWB_valid),
        .MEMWB_flags(MEMWB_flags),
        .ccr_out(ccr_out)
    );
    
    // ==================== STAGE LOGIC ====================
    
    // Initialize
    initial begin
        flush = 1'b0;
        branch_taken = 1'b0;
        interrupt_pending = 1'b0;
        in_interrupt = 1'b0;
        out_port = 8'h00;
    end
    
    // Fetch stage logic
    always @(posedge clk) begin
        if (reset) begin
            flush <= 1'b0;
            branch_taken <= 1'b0;
        end
        else if (interrupt && !in_interrupt && !interrupt_pending) begin
            interrupt_pending <= 1'b1;
            interrupt_return_pc <= pc_out;
            interrupt_flags <= ccr_out;
            flush <= 1'b1;
        end
        else if (!stall) begin
            if (flush) begin
                flush <= 1'b0;
            end
            if (branch_taken) begin
                branch_taken <= 1'b0;
            end
        end
    end
    
    // Decode Stage (Combinational)
    always @(*) begin
        decode_opcode = IFID_instruction[7:4];
        decode_ra = IFID_instruction[3:2];
        decode_rb = IFID_instruction[1:0];
        decode_valid = IFID_valid;
        decode_is_two_byte = ctrl_is_two_byte;
        
        // Get operands with forwarding
        // Special case for stack operations
        if (ctrl_is_stack_op) begin
            if (forward_ex_valid && forward_ex_reg == 2'b11)
                decode_operand_a = forward_ex_data;
            else if (forward_mem_valid && forward_mem_reg == 2'b11)
                decode_operand_a = forward_mem_data;
            else
                decode_operand_a = reg_read_data1; // SP
        end
        else begin
            if (forward_ex_valid && forward_ex_reg == IFID_instruction[3:2])
                decode_operand_a = forward_ex_data;
            else if (forward_mem_valid && forward_mem_reg == IFID_instruction[3:2])
                decode_operand_a = forward_mem_data;
            else
                decode_operand_a = reg_read_data1;
        end
        
        // operand_b: Special handling for POP and IN
        if (IFID_instruction[7:4] == 4'h7 && (IFID_instruction[3:2] == 2'b01 || IFID_instruction[3:2] == 2'b11)) begin
            decode_operand_b = 8'h00;
        end
        else begin
            if (forward_ex_valid && forward_ex_reg == IFID_instruction[1:0])
                decode_operand_b = forward_ex_data;
            else if (forward_mem_valid && forward_mem_reg == IFID_instruction[1:0])
                decode_operand_b = forward_mem_data;
            else
                decode_operand_b = reg_read_data2;
        end
        
        // Get immediate for 2-byte instructions
        if (ctrl_is_two_byte) begin
            decode_imm = imem_inst.memory[IFID_pc + 1];
        end
        else begin
            decode_imm = 8'h00;
        end
    end
    
    // Execute Stage (Combinational)
    always @(*) begin
        execute_result = alu_result;
        execute_address = 8'h00;
        execute_flags = alu_flags;
        execute_valid = IDEX_valid;
        take_branch = 1'b0;
        branch_target = 8'h00;
        branch_taken = 1'b0;
        
        if (IDEX_valid) begin
            case (IDEX_opcode)
                4'h7: begin // PUSH, POP, OUT, IN
                    case (IDEX_ra)
                        2'b00: begin // PUSH
                            execute_address = IDEX_operand_a;
                            execute_result = IDEX_operand_b;
                        end
                        2'b01: begin // POP
                            execute_address = IDEX_operand_a + 1;
                        end
                        2'b10: begin // OUT
                            execute_result = IDEX_operand_b;
                        end
                        2'b11: begin // IN
                            execute_result = in_port;
                        end
                    endcase
                end
                
                4'h9: begin // JZ, JN, JC, JV
                    case (IDEX_ra)
                        2'b00: take_branch = EXMEM_flags[0] | ccr_out[0];
                        2'b01: take_branch = EXMEM_flags[1] | ccr_out[1];
                        2'b10: take_branch = EXMEM_flags[2] | ccr_out[2];
                        2'b11: take_branch = EXMEM_flags[3] | ccr_out[3];
                    endcase
                    if (take_branch) begin
                        branch_target = IDEX_operand_b;
                        branch_taken = 1'b1;
                    end
                end
                
                4'hA: begin // LOOP
                    if (alu_result != 8'h00) begin
                        branch_target = IDEX_operand_b;
                        branch_taken = 1'b1;
                    end
                end
                
                4'hB: begin // JMP, CALL, RET, RTI
                    case (IDEX_ra)
                        2'b00: begin // JMP
                            branch_target = IDEX_operand_b;
                            branch_taken = 1'b1;
                        end
                        2'b01: begin // CALL
                            execute_address = IDEX_operand_a;
                            execute_result = IDEX_pc + 1;
                            branch_target = IDEX_operand_b;
                            branch_taken = 1'b1;
                        end
                        2'b10, 2'b11: begin // RET, RTI
                            execute_address = IDEX_operand_a + 1;
                            branch_target = dmem_inst.memory[IDEX_operand_a + 1];
                            branch_taken = 1'b1;
                        end
                    endcase
                end
                
                4'hC: begin // LDM, LDD, STD
                    case (IDEX_ra)
                        2'b00: execute_result = IDEX_imm; // LDM
                        2'b01, 2'b10: begin // LDD, STD
                            execute_address = IDEX_imm;
                            if (IDEX_ra == 2'b10)
                                execute_result = IDEX_operand_b;
                        end
                    endcase
                end
                
                4'hD, 4'hE: begin // LDI, STI
                    execute_address = IDEX_operand_a;
                    if (IDEX_opcode == 4'hE)
                        execute_result = IDEX_operand_b;
                end
            endcase
        end
    end
    
    // Memory Stage (Combinational)
    always @(*) begin
        memory_data = 8'h00;
        memory_dest_reg = EXMEM_rb;
        memory_write_enable = 1'b0;
        memory_update_sp = 1'b0;
        is_decre=1'b0;
        memory_new_sp = 8'h00;
        memory_valid = EXMEM_valid;
        data_mem_write_en = 1'b0;
        data_mem_write_addr = 8'h00;
        data_mem_write_data = 8'h00;
        
        if (EXMEM_valid) begin
            // Use control signals from EX/MEM
            memory_write_enable = EXMEM_ctrl_reg_write;
            
            case (EXMEM_opcode)
                4'h1, 4'h2, 4'h3, 4'h4, 4'h5: begin // MOV, ADD, SUB, AND, OR
                    memory_data = EXMEM_result;
                    memory_dest_reg = EXMEM_ra;
                end
                
                4'h6: begin // RLC, RRC
                    if (EXMEM_ra <= 2'b01) begin
                        memory_data = EXMEM_result;
                        memory_dest_reg = EXMEM_rb;
                    end
                end
                
                4'h7: begin // PUSH, POP, OUT, IN
                    case (EXMEM_ra)
                        2'b00: begin // PUSH
                            data_mem_write_en = 1'b1;
                            data_mem_write_addr = EXMEM_address;
                            data_mem_write_data = EXMEM_result;
                            memory_update_sp = 1'b1;
                            is_decre=1'b1;
                            memory_new_sp = EXMEM_address - 1;
                        end
                        2'b01: begin // POP
                            memory_data = data_mem_read;
                            memory_dest_reg = EXMEM_rb;
                            memory_update_sp = 1'b1;
                            is_decre=1'b0;
                            memory_new_sp = EXMEM_address;
                        end
                        2'b10: begin // OUT
                            out_port = EXMEM_result;
                        end
                        2'b11: begin // IN
                            memory_data = EXMEM_result;
                            memory_dest_reg = EXMEM_rb;
                        end
                    endcase
                end
                
                4'h8: begin // NOT, NEG, INC, DEC
                    memory_data = EXMEM_result;
                end
                
                4'hA: begin // LOOP
                    memory_data = EXMEM_result;
                    memory_dest_reg = EXMEM_ra;
                end
                
                4'hB: begin // CALL, RET, RTI
                    case (EXMEM_ra)
                        2'b01: begin // CALL
                            data_mem_write_en = 1'b1;
                            data_mem_write_addr = EXMEM_address;
                            data_mem_write_data = EXMEM_result;
                            memory_update_sp = 1'b1;
                            is_decre=1'b1;
                            memory_new_sp = EXMEM_address - 1;
                        end
                        2'b10, 2'b11: begin // RET, RTI
                            memory_update_sp = 1'b1;
                            is_decre=1'b0;
                            memory_new_sp = EXMEM_address;
                        end
                    endcase
                end
                
                4'hC: begin // LDM, LDD, STD
                    case (EXMEM_ra)
                        2'b00, 2'b01: begin // LDM, LDD
                            memory_data = (EXMEM_ra == 2'b00) ? EXMEM_result : data_mem_read;
                            memory_dest_reg = EXMEM_rb;
                        end
                        2'b10: begin // STD
                            data_mem_write_en = 1'b1;
                            data_mem_write_addr = EXMEM_address;
                            data_mem_write_data = EXMEM_result;
                        end
                    endcase
                end
                
                4'hD: begin // LDI
                    memory_data = data_mem_read;
                end
                
                4'hE: begin // STI
                    data_mem_write_en = 1'b1;
                    data_mem_write_addr = EXMEM_address;
                    data_mem_write_data = EXMEM_result;
                end
            endcase
        end
    end
    
    // Interrupt handling
    always @(posedge clk) begin
        if (reset) begin
            interrupt_pending <= 1'b0;
            in_interrupt <= 1'b0;
        end
        else if (interrupt_pending) begin
            dmem_inst.memory[regfile_inst.registers[3]] <= interrupt_return_pc;
            in_interrupt <= 1'b1;
            interrupt_pending <= 1'b0;
        end
    end

endmodule