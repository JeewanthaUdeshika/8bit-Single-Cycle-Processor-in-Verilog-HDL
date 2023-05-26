/*
CO224  Lab 05
Part 05 - Fully functional CPU with 3 additional instructions
Group No - 07

E/18/285 - Ranasinghe S.M.T.S.C.
E/18/028 - Ariyawansha P.H.J.U.
*/

`timescale 1ns/100ps

`include "alu.v"
`include "register.v"
`include "DataMemory2.v"
`include "dcacheFSM_v2.v"

// This is the control unit for give signals to elements according to given instruction's OPCODE
module control_unit (OPCODE, ALUOP, SIGN, IMMEDIATE, WRITEENABLE, PC_SELECTOR, WRITE, READ, DATAMEM, BUSYWAIT);
    input[7:0] OPCODE;
    input BUSYWAIT;
    output reg[2:0] ALUOP;
    output reg SIGN, IMMEDIATE, WRITEENABLE, WRITE, READ, DATAMEM;
    output reg[1:0] PC_SELECTOR;

    always @(BUSYWAIT) begin
        if (BUSYWAIT == 1'b0) begin
            assign WRITE = 1'b0;
            assign READ = 1'b0;  
        end    
    end

    always@(OPCODE) begin               // TODO: Sensitivity list - PC
        case(OPCODE)
        8'b00000010: begin// add
        #1
            assign ALUOP = 3'b001;
            assign SIGN = 1'b0;         
            assign WRITEENABLE = 1'b1;  // Writeenable must be 1 to add sub and or move
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
            assign WRITE = 1'b0;
            assign READ = 1'b0;
            assign DATAMEM = 1'b0;
        end

        8'b00000011: begin  // sub
        #1
            assign ALUOP = 3'b001;
            assign SIGN = 1'b1;         // for sub operands should be in signed 
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
            assign WRITE = 1'b0;
            assign READ = 1'b0;
            assign DATAMEM = 1'b0;
        end

        8'b00000100: begin  // AND
        #1
            assign ALUOP = 3'b010;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
            assign WRITE = 1'b0;
            assign READ = 1'b0;
            assign DATAMEM = 1'b0;
        end

        8'b00000101: begin  // OR
        #1
            assign ALUOP = 3'b011;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
            assign WRITE = 1'b0;
            assign READ = 1'b0;
            assign DATAMEM = 1'b0;
        end

        8'b00000001: begin  // MOV
        #1
            assign ALUOP = 3'b000;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
            assign PC_SELECTOR = 2'b00;
            assign WRITE = 1'b0;
            assign READ = 1'b0;
            assign DATAMEM = 1'b0;
        end

        8'b00000000: begin  // LOADI (here immidiate should be 1)
        #1
            assign ALUOP = 3'b000;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b1;
            assign PC_SELECTOR = 2'b00;
            assign WRITE = 1'b0;
            assign READ = 1'b0;
            assign DATAMEM = 1'b0;
        end

        8'b00000111: begin // BEQ
        #1
          assign ALUOP = 3'b001;
          assign SIGN = 1'b1;
          assign WRITEENABLE = 1'b0;
          assign IMMEDIATE = 1'b0;
          assign PC_SELECTOR = 2'b01;
          assign WRITE = 1'b0;
          assign READ = 1'b0;
          assign DATAMEM = 1'b0;
        end

        8'b00000110: begin // jump
        #1
          assign ALUOP = 3'b001;
          assign SIGN = 1'b0;
          assign WRITEENABLE = 1'b0;
          assign IMMEDIATE = 1'b0;
          assign PC_SELECTOR = 2'b10;
          assign WRITE = 1'b0;
          assign READ = 1'b0;
          assign DATAMEM = 1'b0;
        end

        8'b00001001: begin // lwi
        #1
        assign ALUOP = 3'b000;
          assign SIGN = 1'b0;
          assign WRITEENABLE = 1'b1;
          assign IMMEDIATE = 1'b1;
          assign PC_SELECTOR = 2'b00;
          assign WRITE = 1'b0;
          assign READ = 1'b1;
          assign DATAMEM = 1'b1;    // If datamem is zero data memory output is not used and ALU result is not an address
        end

        8'b00001000: begin // lwd
        #1
        assign ALUOP = 3'b000;
          assign SIGN = 1'b0;
          assign WRITEENABLE = 1'b1;
          assign IMMEDIATE = 1'b0;
          assign PC_SELECTOR = 2'b00;
          assign WRITE = 1'b0;
          assign READ = 1'b1;
          assign DATAMEM = 1'b1;    // If datamem is zero data memory output is not used and ALU result is not an address
        end

        8'b00001010: begin // swd
        #1
        assign ALUOP = 3'b000;
          assign SIGN = 1'b0;
          assign WRITEENABLE = 1'b0;
          assign IMMEDIATE = 1'b0;
          assign PC_SELECTOR = 2'b00;
          assign WRITE = 1'b1;
          assign READ = 1'b0;
          assign DATAMEM = 1'b1;    // If datamem is zero data memory output is not used and ALU result is not an address
        end

        8'b00001011: begin // swi
        #1
        assign ALUOP = 3'b000;
          assign SIGN = 1'b0;
          assign WRITEENABLE = 1'b0;
          assign IMMEDIATE = 1'b1;
          assign PC_SELECTOR = 2'b00;
          assign WRITE = 1'b1;
          assign READ = 1'b0;
          assign DATAMEM = 1'b1;    // If datamem is zero data memory output is not used and ALU result is not an address
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
        //default:
           // assign result = 8'bx;
        endcase

    end
    
endmodule

// PC module for CPU
module PCpro(PC, EXTENDED_OFFSET, PC_SELECTOR, CLK, RESET, ZERO, BUSYWAIT);
    output reg[31:0] PC;
    input [31:0] EXTENDED_OFFSET;
    input [1:0] PC_SELECTOR;
    input CLK, RESET, ZERO, BUSYWAIT;
    reg [31:0] temp, PC_Temp, PC_Next, target;

    always @(EXTENDED_OFFSET) begin
        temp = EXTENDED_OFFSET;
    end

    always @(temp) begin
        #2 target = PC_Temp + temp;
    end

    always @(PC) begin
        #1 PC_Temp = PC + 4;
    end

    always @(PC_SELECTOR, ZERO, PC_Temp, target) begin
        if (PC_SELECTOR == 2'b10) begin  // If instruction is jump, add 4 and offset to PC
            PC_Next = target;
        end

        else if (PC_SELECTOR == 2'b01 && ZERO == 1'b1) begin    // If instruction is beq, check alu output is zero and add 4 and offset to the PC
            PC_Next = target;
        end

        else begin
            PC_Next = PC_Temp;      // If opcode is neither jump nor beq, add only 4 to PC
        end
    end

    always @(posedge CLK) begin
        if (RESET == 1'b1) begin
            #1 PC = 32'b00000000000000000000000000000000;  // Reset pc if reset is high
        end

        else if (!BUSYWAIT) begin
            PC = PC_Next;
        end

       // else begin
       //     #1 PC = PC_Next;
       // end
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
    
    wire WRITEENABLE, SIGN, IMMEDIATE, ShiftSignal, BNE;
    wire [2:0] ALUOP;

    wire [7:0] ToMUX2, ALUIN1;
    wire [7:0] negative;
    wire ZERO;

    wire [7:0] READDATA, WRITEDATA;
    wire WRITE, READ, DATAMEM, BUSYWAIT;

    wire mem_read, mem_write, mem_busywait;
    wire [31:0] mem_writedata, mem_readdata;
    wire [5:0] mem_address;


    // decording the instructions
    always @(INSTRUCTION) begin
        OPCODE = INSTRUCTION[31:24];
        WRITEREG = INSTRUCTION[18:16];
        READREG1 = INSTRUCTION[10:8];
        READREG2 = INSTRUCTION[2:0];
        IMMEDIATEVal = INSTRUCTION[7:0];
        OFFSET = INSTRUCTION[23:16];
    end

    // PC module
    PCpro pc1(PC, SHIFTED, PC_SELECTOR, CLK, RESET, ZERO, BUSYWAIT);

    // Instantiation Control unit module
    control_unit control(OPCODE, ALUOP, SIGN, IMMEDIATE, WRITEENABLE, PC_SELECTOR, WRITE, READ, DATAMEM, BUSYWAIT);

    // Extend sign in off set
    signEx signEx1(OFFSET, EXTENDED);

    // Shift left before initializing Branch adder
    shiftLeft shiftLeft1(EXTENDED, SHIFTED);

    // Instiation Register file module
    reg_file regfile(WRITEDATA, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);

    // Instatiation 2s complement module
    twos_Comp twos_comp1(REGOUT2, negative);

    // Institation MUX for 2s complement operation
    MUX_2x1 mux4sign(REGOUT2, negative, SIGN, ToMUX2);

    // Institation MUX for Immediate values
    MUX_2x1 mux4Ime(ToMUX2, IMMEDIATEVal, IMMEDIATE , ALUIN1);

    // Instiation ALU module
    ALU ALU1(REGOUT1, ALUIN1, ALUOUT, ALUOP, ZERO);

    // Instantiation DataMemory
    data_memory dmem1(
        CLK,
        RESET,
        mem_read,
        mem_write,
        mem_address,
        mem_writedata,
        mem_readdata,
        mem_busywait
    );

    // Instantiation Cache Memory
    dcache dcache1(
        CLK,
        RESET,
        READ,
        WRITE,
        mem_read,
        mem_write,
        ALUOUT,
        mem_address,
        REGOUT1,
        mem_writedata,
        READDATA,
        mem_readdata,
        BUSYWAIT,
        mem_busywait
    );

    // Mux to select what is write
    MUX_2x1 writeSelect(ALUOUT, READDATA, DATAMEM, WRITEDATA);
 
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
        /* {instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000_00000100_00000000_00000101;            // loadi 4 0x05
        {instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000_00000101_00000000_00000011;            // loadi 5 0x03
        {instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000000_00000011_00000000_00000001;          // loadi 3 0x01
        {instr_mem[10'd15], instr_mem[10'd14], instr_mem[10'd13], instr_mem[10'd12]} = 32'b00001011_00000000_00000100_00000000;        // swi 4 0x00        
        {instr_mem[10'd19], instr_mem[10'd18], instr_mem[10'd17], instr_mem[10'd16]} = 32'b00001010_00000000_00000101_00000011;        // swd 5 3
        {instr_mem[10'd23], instr_mem[10'd22], instr_mem[10'd21], instr_mem[10'd20]} = 32'b00000010_00000100_00000100_00000101;        // add 4 4 5
        {instr_mem[10'd27], instr_mem[10'd26], instr_mem[10'd25], instr_mem[10'd24]} = 32'b00000011_00000101_00000100_00000101;        // sub 5 4 5
        {instr_mem[10'd31], instr_mem[10'd30], instr_mem[10'd29], instr_mem[10'd28]} = 32'b00001001_00000110_00000000_00000000;        // lwi 6 0x00
        {instr_mem[10'd35], instr_mem[10'd34], instr_mem[10'd33], instr_mem[10'd32]} = 32'b00001000_00000111_00000000_00000011;        // lwd 7 3
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
        RESET = 1'b0;
        #2
        RESET = 1'b1;
        #10
        RESET = 1'b0;
        
        // finish simulation after some time
        #1000
        $finish;
        
    end

    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule