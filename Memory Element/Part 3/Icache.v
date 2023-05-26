/*
CO224  Lab 06
Part 03 - Instruction Cache and Memory
Group No - 07

E/18/285 - Ranasinghe S.M.T.S.C.
E/18/028 - Ariyawansha P.H.J.U.

This file presents a skeleton implementation of the cache controller using a Finite State Machine model.
*/

`timescale 1ns/100ps

module Icache (
    clock,
    reset,
    read,
    address,
    mem_address,
    readInstruction,
    mem_readInstruction,
    busywait,
    mem_busywait
);

    input clock, mem_busywait, reset;
    input [9:0] address;                // Address from cpu
    input [127:0] mem_readInstruction;    // istruction from memory
    output reg read, busywait;          // Read signal to instruction memory
    output reg [5:0] mem_address;       // Address for instruction memory
    output reg [31:0] readInstruction;       // Instruction for cpu

    reg [2:0] index, tag, c_tag;
    reg [3:0] offset;
    reg [31:0] instruction1, instruction2, instruction3, instruction4;
    reg hit, valid;
    integer i;



    // Cache memory array
    reg[131:0] cache[7:0];              // valid[131] | Tag[130:128] | Data[127:0]


    // Spliting address into parts
    always @(address, mem_readInstruction, cache[index]) begin
        
        // spliting the input address
        #1
        offset  = address[3:0];
        index   = address[6:4];
        tag     = address[9:7];

        valid = cache[index][131];
        c_tag   = cache[index][130:128];

        instruction1 = cache[index][31:0];
        instruction2 = cache[index][63:32];
        instruction3 = cache[index][95:64];
        instruction4 = cache[index][127:96];

    end

    // Tag comparison and validation
    always @(valid, tag, c_tag) begin
        if (valid == 1'b1 && tag == c_tag) begin
            #0.9 hit = 1'b1;    // HIT
        end
        else begin
            #0.9 hit = 1'b0;    // MISS
        end

    end

    // Returning the read instruction
    always @(instruction1, instruction2, instruction3, instruction4, offset) begin
        #1
        case (offset)
            4'b0000:
                readInstruction = instruction1;
            4'b0100:
                readInstruction = instruction2;
            4'b1000:
                readInstruction = instruction3;
            4'b1100:
                readInstruction = instruction4; 
        endcase
    end

    // If it is hit, set busywait signal
    always @(*) begin
        if (hit) begin
            busywait = 0;
        end
        else begin
            busywait = 1;
        end
    end

    /* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001, FETCH = 3'b111;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if (!hit) begin
                    next_state = MEM_READ;
                end

                else begin
                    next_state = IDLE;
                end
            
            MEM_READ:
                if (!mem_busywait) begin
                    next_state = FETCH;
                end

                else begin    
                    next_state = MEM_READ;
                end
            
            FETCH:  // Here in this state, all the data from data memory is written in cache according to the index
                next_state = IDLE;
            
        endcase
    end

    // combinational output logic
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                read = 0;
                mem_address = 8'dx;
                busywait = 1'b0;
            end
            
            MEM_READ: 
            begin
                read = 1;
                mem_address = {tag, index};
                busywait = 1'b1;
            end

            FETCH:
            begin
                read = 0;
                mem_address = 6'bx;
                busywait = 1'b1;

            #1  cache[index][127:0] = mem_readInstruction;	//write a data block to the cache
                cache[index][130:128] = tag;	//tag C_Address[9:7]
                cache[index][131] = 1'b1;	//valid bit

            end

            
        endcase
    end

    // sequential logic for state transitioning 
    always @(posedge clock, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

    /* Cache Controller FSM End */

endmodule

