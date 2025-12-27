// hazard_unit.v - Hazard Detection Unit (FIXED FOR EX-ID RAW HAZARD)
// Now detects and stalls for EX-ID RAW hazards

module hazard_unit(
    input wire IFID_valid,
    input wire [7:0] IFID_instruction,
    input wire IDEX_valid,
    input wire [3:0] IDEX_opcode,
    input wire [1:0] IDEX_ra,
    input wire [1:0] IDEX_rb,
    
    output reg stall
);

    reg load_use_hazard;
    reg ex_id_hazard;  // NEW: Detects EX-ID RAW hazard
    
    always @(*) begin
        load_use_hazard = 1'b0;
        ex_id_hazard = 1'b0;
        stall = 1'b0;
        
        // ========================================
        // EX-ID RAW Hazard Detection (NEW - FIXES THE BUG!)
        // When instruction in EX writes to register that instruction in ID reads
        // This causes the "R0=2 instead of 5" bug
        // ========================================
        if (IDEX_valid && IFID_valid) begin
            // Check if IDEX instruction writes to a register
            case (IDEX_opcode)
                4'h1, 4'h2, 4'h3, 4'h4, 4'h5: begin // MOV, ADD, SUB, AND, OR
                    // These write to IDEX_ra
                    // Check if IFID reads from this register
                    if (IFID_instruction[3:2] == IDEX_ra || IFID_instruction[1:0] == IDEX_ra) begin
                        // Check if IFID actually uses this register as SOURCE
                        case (IFID_instruction[7:4])
                            4'h0: ex_id_hazard = 1'b0; // NOP
                            4'h1: begin // MOV uses Rb only
                                if (IFID_instruction[1:0] == IDEX_ra)
                                    ex_id_hazard = 1'b1;
                            end
                            4'h2, 4'h3, 4'h4, 4'h5: ex_id_hazard = 1'b1; // ADD/SUB/AND/OR use Ra and Rb
                            4'h8: begin // NOT/NEG/INC/DEC use Rb
                                if (IFID_instruction[1:0] == IDEX_ra)
                                    ex_id_hazard = 1'b1;
                            end
                            4'h9: begin // Branches use Rb
                                if (IFID_instruction[1:0] == IDEX_ra)
                                    ex_id_hazard = 1'b1;
                            end
                            4'hA: ex_id_hazard = 1'b1; // LOOP uses Ra and Rb
                        endcase
                    end
                end
                
                4'h8: begin // NOT/NEG/INC/DEC write to Rb
                    if (IFID_instruction[3:2] == IDEX_rb || IFID_instruction[1:0] == IDEX_rb) begin
                        case (IFID_instruction[7:4])
                            4'h0: ex_id_hazard = 1'b0;
                            4'h1: begin
                                if (IFID_instruction[1:0] == IDEX_rb)
                                    ex_id_hazard = 1'b1;
                            end
                            4'h2, 4'h3, 4'h4, 4'h5, 4'h8: ex_id_hazard = 1'b1;
                            default: ex_id_hazard = 1'b0;
                        endcase
                    end
                end
                
                4'hA: begin // LOOP writes to Ra
                    if (IFID_instruction[3:2] == IDEX_ra || IFID_instruction[1:0] == IDEX_ra) begin
                        case (IFID_instruction[7:4])
                            4'h0: ex_id_hazard = 1'b0;
                            default: ex_id_hazard = 1'b1;
                        endcase
                    end
                end
            endcase
        end
        
        // ========================================
        // Consecutive Stack Operations
        // ========================================
        if (IDEX_valid && (IDEX_opcode == 4'h7 || IDEX_opcode == 4'hB)) begin
            if (IFID_valid && (IFID_instruction[7:4] == 4'h7 || IFID_instruction[7:4] == 4'hB)) begin
                load_use_hazard = 1'b1;
            end
        end
        
        // ========================================
        // Load-Use Hazard Detection
        // ========================================
        if (IDEX_valid) begin
            case (IDEX_opcode)
                4'hC: begin // LDM, LDD, STD
                    if (IDEX_ra == 2'b00 || IDEX_ra == 2'b01) begin // LDM or LDD
                        if (IFID_valid) begin
                            if (IFID_instruction[7:4] !=4'hc) begin
                            if (IFID_instruction[3:2] == IDEX_rb || IFID_instruction[1:0] == IDEX_rb) begin
                                case (IFID_instruction[7:4])
                                    4'h0: load_use_hazard = 1'b0;
                                    4'h6: begin
                                        if (IFID_instruction[3:2] >= 2'b10)
                                            load_use_hazard = 1'b0;
                                        else
                                            load_use_hazard = 1'b1;
                                    end
                                    4'h7: load_use_hazard = 1'b1;
                                    default: load_use_hazard = 1'b1;
                                endcase
                            end
                        end
                        end
                    end
                end
                
                4'hD: begin // LDI
                    if (IFID_valid) begin
                        if (IFID_instruction[3:2] == IDEX_rb || IFID_instruction[1:0] == IDEX_rb) begin
                            case (IFID_instruction[7:4])
                                4'h0: load_use_hazard = 1'b0;
                                4'h6: begin
                                    if (IFID_instruction[3:2] >= 2'b10)
                                        load_use_hazard = 1'b0;
                                    else
                                        load_use_hazard = 1'b1;
                                end
                                default: load_use_hazard = 1'b1;
                            endcase
                        end
                    end
                end
                
                4'h7: begin // POP, IN
                    if (IDEX_ra == 2'b01 || IDEX_ra == 2'b11) begin
                        if (IFID_valid) begin
                            if (IFID_instruction[3:2] == IDEX_rb || IFID_instruction[1:0] == IDEX_rb) begin
                                case (IFID_instruction[7:4])
                                    4'h0: load_use_hazard = 1'b0;
                                    4'h6: begin
                                        if (IFID_instruction[3:2] >= 2'b10)
                                            load_use_hazard = 1'b0;
                                        else
                                            load_use_hazard = 1'b1;
                                    end
                                    default: load_use_hazard = 1'b1;
                                endcase
                            end
                        end
                    end
                end
            endcase
        end
        
        // Set stall if ANY hazard detected
        stall = load_use_hazard | ex_id_hazard;
    end

endmodule