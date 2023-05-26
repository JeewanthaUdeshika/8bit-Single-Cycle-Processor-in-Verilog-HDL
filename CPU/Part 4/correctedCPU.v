/*
CO224  Lab 05
Part 04 - Fully functional CPU
Group No - 07

E/18/285 - Ranasinghe S.M.T.S.C.
E/18/028 - Ariyawansha P.H.J.U.
*/

`include "alu.v"
`include "register.v"

// This is the control unit for give signals to elements according to given instruction's OPCODE
module control_unit (OPCODE, ALUOP, SIGN, IMMEDIATE, WRITEENABLE, PC_SELECTOR);
    input[7:0] OPCODE;
    output reg[2:0] ALUOP;
    output reg SIGN, IMMEDIATE, WRITEENABLE;
    output reg[1:0] PC_SELECTOR;

    always@(OPCODE) begin
        case(OPCODE)
        8'b00000010: begin// add
        #1
            assign ALUOP = 3'b001;
            assign SIGN = 1'b0;         
            assign WRITEENABLE = 1'b1;  // Writeenable must be 1 to add sub and or move
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
        end

        8'b00000011: begin  // sub
        #1
            assign ALUOP = 3'b001;
            assign SIGN = 1'b1;         // for sub operands should be in signed 
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00; 
        end

        8'b00000100: begin  // AND
        #1
            assign ALUOP = 3'b010;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
        end

        8'b00000101: begin  // OR
        #1
            assign ALUOP = 3'b011;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
        end

        8'b00000001: begin  // MOV
        #1
            assign ALUOP = 3'b000;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
        end

        8'b00000000: begin  // LOADI (here immidiate should be 1)
        #1
            assign ALUOP = 3'b000;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b1;
            assign PC_SELECTOR = 2'b00;
        end

        8'b00000111: begin // BEQ
        #1
          assign ALUOP = 3'b001;
          assign SIGN = 1'b1;
          assign WRITEENABLE = 1'b0;
          assign IMMEDIATE = 1'b0;
          assign PC_SELECTOR = 2'b01;    
        end

        8'b00000110: begin // jump
        #1
          assign ALUOP = 3'b001;
          assign SIGN = 1'b0;
          assign WRITEENABLE = 1'b0;
          assign IMMEDIATE = 1'b0;
          assign PC_SELECTOR = 2'b10;
        end

        endcase 
    end

endmodule

// This is the 2s complement to sum
module twos_Comp (unsign, sign);
    input [7:0] unsign;
    output reg [7:0] sign;
    
    always @(unsign) begin
        #1 sign = ~unsign + 8'b00000001;  
    end
endmodule

// This is a 2 to 1 MUX for usages in cpu
module MUX_2x1 (data1, data2, select, result);
    input [7:0] data1, data2;
    input select;
    output reg[7:0] result;

    always @(data1, data2, select) begin
        case (select)
        1'b0:
            assign result = data1;
        1'b1:
            assign result = data2;
        default:
            assign result = 8'bx;
        endcase

    end
    
endmodule

// PC updating module for CPU
module pc(PC_VAL, CLK, RESET, PC);
    input CLK, RESET;
    input [31:0] PC_VAL;
    output reg [31:0] PC;

    always @(posedge CLK)
    begin
        if (RESET)
        begin
            #1 PC = 0;
        end

        else begin
            #1 PC = PC_VAL;
        end
    end

endmodule

// This module is selecting the PC according to OPcode is beq, jump or any other
module Target (PCSELECTOR, ZERO, PC, EXTENDED_OFFSET, PC_VAL);
    input [31:0] PC;
    input [1:0] PCSELECTOR;
    input [31:0] EXTENDED_OFFSET;
    input ZERO;
    output reg[31:0] PC_VAL;
    reg[31:0] ADDED_PC;


    always @(PCSELECTOR, EXTENDED_OFFSET, PC, ZERO) begin
        if (PCSELECTOR == 2'b10) begin  // If instruction is jump, add 4 and offset to PC
            #2 PC_VAL = PC + EXTENDED_OFFSET + 4;
        end
        else if(PCSELECTOR == 2'b01 && ZERO == 1'b1) begin  // If instruction is beq, check alu output is zero and add 4 and offset to the PC
            #2 PC_VAL = PC + EXTENDED_OFFSET + 4;   
        end
        else begin
            PC_VAL = PC + 4; // If opcode is neither jump nor beq, add only 4 to PC
        end
    end
endmodule


// Sign extender module
module signEx (toExtend, extended);
    input[7:0] toExtend;
    output reg[31:0] extended;

    always @(toExtend) begin
        if (toExtend[7] == 1'b0) begin
            extended[31:8] <= 24'b0000000000000000000000;
            extended[7:0] <= toExtend;
        end

        else if (toExtend[7] == 1'b1) begin
            extended[31:8] <= 24'b1111111111111111111111;
            extended[7:0] <= toExtend;
        end
    end
endmodule

// module for shift left the output by 2
module shiftLeft (shift, shifted);
    input[31:0] shift;
    output signed[31:0] shifted;

    assign shifted = shift <<< 2;

endmodule

// 32 bit input mux
module MUX_2x1_32 (input1, input2, select, result);
    input [31:0] input1, input2;
    input select;
    output reg [31:0] result;

    always @(input1, input2, select) begin
        case(select)
        1'b0:
            assign result = input1;
        1'b1:
            assign result = input2;
        default:
            assign result = input1;
        endcase
    end
endmodule

// This is the CPU module where all the things happenning
module cpu (PC, INSTRUCTION, CLK, RESET);
    input [31:0] INSTRUCTION;
    input CLK, RESET;
    output [31:0] PC;

    wire [31:0] increPC, EXTENDED, SHIFTED, PC_VAL, branchPC;
    wire [1:0] PC_SELECTOR;

    reg [7:0] OPCODE, IMMEDIATEVal, OFFSET; 
    reg [2:0] READREG1, READREG2, WRITEREG;

    wire [7:0] REGOUT1, REGOUT2;
    wire [7:0] ALUOUT;
    
    wire WRITEENABLE, SIGN, IMMEDIATE;
    wire [2:0] ALUOP;

    wire [7:0] ToMUX2, ALUIN1;
    wire [7:0] negative;
    wire ZERO;


    // decording the instructions
    always @(INSTRUCTION) begin
        OPCODE = INSTRUCTION[31:24];
        WRITEREG = INSTRUCTION[18:16];
        READREG1 = INSTRUCTION[10:8];
        READREG2 = INSTRUCTION[2:0];
        IMMEDIATEVal = INSTRUCTION[7:0];
        OFFSET = INSTRUCTION[23:16];
    end

    // Select PC Value
    Target target1(PC_SELECTOR, ZERO, PC, SHIFTED, PC_VAL);

    // Updating PC
    pc pc1 (PC_VAL, CLK, RESET, PC);

    // Instantiation Control unit module
    control_unit control(OPCODE, ALUOP, SIGN, IMMEDIATE, WRITEENABLE, PC_SELECTOR);

    // Extend sign in off set
    signEx signEx1(OFFSET, EXTENDED);

    // Shift left before initializing Branch adder
    shiftLeft shiftLeft1(EXTENDED, SHIFTED);

    // Instiation Register file module
    reg_file regfile(ALUOUT, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);

    // Instatiation 2s complement module
    twos_Comp twos_comp1(REGOUT2, negative);

    // Institation MUX for 2s complement operation
    MUX_2x1 mux4sign(REGOUT2, negative, SIGN, ToMUX2);

    // Institation MUX for Immediate values
    MUX_2x1 mux4Ime(ToMUX2, IMMEDIATEVal, IMMEDIATE , ALUIN1);

    // Instiation ALU module
    ALU ALU1(REGOUT1, ALUIN1, ALUOUT, ALUOP, ZERO);
 
endmodule


// This the testbench of the CPU
module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;
    
    
    //------------------------
    // SIMPLE INSTRUCTION MEM
    //------------------------
    
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    reg [7:0] instr_mem[1023:0];
    
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
    always @(PC) begin
        #2
        INSTRUCTION = {instr_mem[PC+3], instr_mem[PC+2], instr_mem[PC+1], instr_mem[PC]};
    end
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        /* {instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;            // loadi 4 5
        {instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;            // loadi 2 9
        {instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;          // add 6 2 4
        {instr_mem[10'd15], instr_mem[10'd14], instr_mem[10'd13], instr_mem[10'd12]} = 32'b00000000000000010000000000000101;        // loadi 1 5
        {instr_mem[10'd19], instr_mem[10'd18], instr_mem[10'd17], instr_mem[10'd16]} = 32'b00000111000000100000000100000100;        // beq 0x2 1 4
        {instr_mem[10'd23], instr_mem[10'd22], instr_mem[10'd21], instr_mem[10'd20]} = 32'b00000000000001000000000000000111;        // loadi 4 7
        {instr_mem[10'd27], instr_mem[10'd26], instr_mem[10'd25], instr_mem[10'd24]} = 32'b00000000000001000000000000001101;        // loadi 4 13
        {instr_mem[10'd31], instr_mem[10'd30], instr_mem[10'd29], instr_mem[10'd28]} = 32'b00000000000001000000000000001111;        // loadi 4 15
        {instr_mem[10'd35], instr_mem[10'd34], instr_mem[10'd33], instr_mem[10'd32]} = 32'b00000000000001000000000000001011;        // loadi 4 11
        {instr_mem[10'd39], instr_mem[10'd38], instr_mem[10'd37], instr_mem[10'd36]} = 32'b00000110000000100000000000001011;        // j 0x2
        {instr_mem[10'd43], instr_mem[10'd42], instr_mem[10'd41], instr_mem[10'd40]} = 32'b00000000000001000000000000101101;        // loadi 4 45
        {instr_mem[10'd47], instr_mem[10'd46], instr_mem[10'd45], instr_mem[10'd44]} = 32'b00000000000001000000000000011111;        // loadi 4 31
        {instr_mem[10'd51], instr_mem[10'd50], instr_mem[10'd49], instr_mem[10'd48]} = 32'b00000000000001000000000000011011;        // loadi 4 27
 */
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    
     
    //-----
    // CPU
    //-----
    
    cpu mycpu(PC, INSTRUCTION, CLK, RESET);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        RESET = 1'b1;
        #10
        RESET = 1'b0;
        
        // finish simulation after some time
        #500
        $finish;
        
    end

    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule