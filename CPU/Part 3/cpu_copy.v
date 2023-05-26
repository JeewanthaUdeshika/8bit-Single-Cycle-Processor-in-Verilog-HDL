
`include "alu.v"
`include "register.v"

// This is the control unit for give signals to elements according to given instruction's OPCODE
module control_unit (OPCODE, ALUOP, SIGN, IMMEDIATE, WRITEENABLE);
    input[7:0] OPCODE;
    output reg[2:0] ALUOP;
    output reg SIGN, IMMEDIATE, WRITEENABLE;

    always@(OPCODE) begin
        case(OPCODE)
        8'b00000010: begin// add
        #1
            assign ALUOP = 3'b001;
            assign SIGN = 1'b0;         
            assign WRITEENABLE = 1'b1;  // Writeenable must be 1 to add sub and or move
            assign IMMEDIATE = 1'b0;
        end

        8'b00000011: begin// sub
        #1
            assign ALUOP = 3'b001;
            assign SIGN = 1'b1;         // for sub operands should be in signed 
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0; 
        end

        8'b00000100: begin// AND
        #1
            assign ALUOP = 3'b010;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
        end

        8'b00000101: begin// OR
        #1
            assign ALUOP = 3'b011;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
        end

        8'b00000001: begin// MOV
        #1
            assign ALUOP = 3'b000;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b0;
        end

        8'b00000000: begin// LOADI (here immidiate should be 1)
        #1
            assign ALUOP = 3'b000;
            assign SIGN = 1'b0;
            assign WRITEENABLE = 1'b1;
            assign IMMEDIATE = 1'b1;
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

// This is the adder module to increment PC value in 4
module adder(PCIN, PCOUT);
    input [31:0] PCIN;
    output reg [31:0] PCOUT;

    always @(PCIN) begin
        #1 PCOUT = PCIN + 4;
    end

endmodule

module pc (CLK , RESET  , PC);
    input CLK ,RESET;
    output reg[31:0] PC ;
    reg [31:0] PC_val; // made a reg for the pc value


    always @(posedge CLK) begin
        if (RESET==1'b1) begin
            #1 PC = 32'b0;
        end else begin
            #1  PC = PC_val ;
        end
        
    end

    always @(PC) begin
        
        #1 PC_val = PC + 32'd4 ; //increase the pc value every time when it is changing
        
    end
endmodule


// This is the CPU module where all the things happenning
module cpu (PC, INSTRUCTION, CLK, RESET);
    input [31:0] INSTRUCTION;
    input CLK, RESET;

    output [31:0] PC;
    wire [31:0] increPC;

    reg [7:0] OPCODE, IMMEDIATEVal; 
    reg [2:0] READREG1, READREG2, WRITEREG;

    wire [7:0] REGOUT1, REGOUT2;
    wire [7:0] ALUOUT;
    
    wire WRITEENABLE, SIGN, IMMEDIATE;
    wire [2:0] ALUOP;

    wire [7:0] ToMUX2, ALUIN1;
    wire [7:0] negative;

    pc pc1 (CLK, RESET, PC);

    // decording the instructions
    always @(INSTRUCTION) begin
        OPCODE = INSTRUCTION[31:24];
        WRITEREG = INSTRUCTION[18:16];
        READREG1 = INSTRUCTION[10:8];
        READREG2 = INSTRUCTION[2:0];
        IMMEDIATEVal = INSTRUCTION[7:0];
    end

    // Instantiation Control unit module
    control_unit control(OPCODE, ALUOP, SIGN, IMMEDIATE, WRITEENABLE);

    // Instiation Register file module
    reg_file regfile(ALUOUT, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);

    // Instatiation 2s complement module
    twos_Comp twos_comp1(REGOUT2, negative);

    // Institation MUX for 2s complement operation
    MUX_2x1 mux4sign(REGOUT2, negative, SIGN, ToMUX2);

    // Institation MUX for Immediate values
    MUX_2x1 mux4Ime(ToMUX2, IMMEDIATEVal, IMMEDIATE , ALUIN1);

    // Instiation ALU module
    ALU ALU1(REGOUT1, ALUIN1, ALUOUT, ALUOP);
 
endmodule

// This the testbench of the CPU
module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;
    
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
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
        //{instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        //{instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        //{instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */
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
