// Testbench for Modular Processor
// Updated to work with separated modules

`timescale 1ns/1ps

module processor_tb;

    reg clk;
    reg reset;
    reg interrupt;
    reg [7:0] in_port;
    wire [7:0] out_port;
    
    // Instantiate processor
    processor_top uut (
        .clk(clk),
        .reset(reset),
        .interrupt(interrupt),
        .in_port(in_port),
        .out_port(out_port)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period = 100MHz
    end
    
    // Test stimulus
    initial begin
        // Initialize signals
        reset = 1;
        interrupt = 0;
        in_port = 8'hbb;
        
        // VCD dump for waveform viewing
        $dumpfile("processor.vcd");
        $dumpvars(0, processor_tb);
        

  //////// add ur program here

               uut.imem_inst.memory[0] = 8'hC0;  // LDM R0, 5
        uut.imem_inst.memory[1] = 8'h05;
        uut.imem_inst.memory[2] = 8'hC1;  // LDM R1, 9
        uut.imem_inst.memory[3] = 8'h09;
        uut.imem_inst.memory[4] = 8'h30;  // sub R0, R0
        uut.imem_inst.memory[5] = 8'h91;  // JZ R1
        uut.imem_inst.memory[6] = 8'h78;  // out R0
        uut.imem_inst.memory[7] = 8'h14;  // Mov R0,R1
        uut.imem_inst.memory[8] = 8'h08; 
        uut.imem_inst.memory[9] = 8'h00;  // NOP



////end ur program

        // Initialize data memory
        uut.dmem_inst.memory[0] = 8'h00;  // Reset vector
        uut.dmem_inst.memory[1] = 8'h00;  // Interrupt vector
        
        
        // Release reset
        #20;
        reset = 0;
        
        // Run for enough cycles
        #400;
        
        // Check results
        // Access registers via module hierarchy: uut.regfile_inst.registers[num]
        $display("\n=== Final Results ===");
        $display("R0 = 0x%02h (expected 0x05)", uut.regfile_inst.registers[0]);
        $display("R1 = 0x%02h (expected 0x03)", uut.regfile_inst.registers[1]);
        $display("R2 = 0x%02h (expected 0x00)", uut.regfile_inst.registers[2]);
        $display("SP = 0x%02h (expected 0xFF)", uut.regfile_inst.registers[3]);
        
        
        $display("\n=== Simulation Complete ===");
        #20;
        $finish;
    end
    

endmodule