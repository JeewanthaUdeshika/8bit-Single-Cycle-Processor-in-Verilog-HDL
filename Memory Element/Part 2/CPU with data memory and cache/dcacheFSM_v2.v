/*
Module  : Data Cache 
Author  : Isuru Nawinne, Kisaru Liyanage
Date    : 25/05/2020

Description	:

This file presents a skeleton implementation of the cache controller using a Finite State Machine model. Note that this code is not complete.
*/

`timescale 1ns/100ps

module dcache (
    clock,
    reset,
    read,
    write,
    mem_read,
    mem_write,
    address,
    mem_address,
    writedata,
    mem_writedata,
    readdata,
    memReaddata,
    busywait,
    mem_busywait
);

    input clock, reset, read, write, mem_busywait;
    input [7:0] address, writedata;
    input [31:0] memReaddata;
    output reg mem_read, mem_write;
    output reg busywait;
    output reg [31:0] mem_writedata;
    output reg [7:0] readdata;
    output reg [5:0] mem_address;

    reg [1:0] OFFSET;
    reg [2:0] index;
    reg [2:0] tag;

    reg valid, dirty, hit;
    reg [2:0] c_tag;
    reg [7:0] c_data1, c_data2, c_data3, c_data4;
    reg [31:0] c_data_block;
    integer i;
    integer j;

    reg readaccess;
    reg writeaccess;

    // Chache memory array
    reg [36:0] cache[7:0];     // Valid[36] | dirty[35] | tag[34:32] | Data[31:0]

    // Asserting the busywait signal
    always @(read, write) begin
        busywait = (read || write)? 1: 0;       // When read or write operation, set busy wait to high
        readaccess = (read && !write)? 1 : 0;   // Check the operation is read
        writeaccess = (!read && write)? 1: 0;   // Check the operation is write
    end

    // When reset = 1, cache is made to reset
    always @(posedge clock, reset) begin

        if(reset == 1'b1) begin

            busywait = 1'b0;    // Make busywait signal is zero when resetting

            // Set 0 to cache memory when reset is high
            #1
            for (i=0; i<8; i = i+1) begin
                    cache[i] <= 32'b0;
            end
        end
    end
    
    /*
    Combinational part for indexing, tag comparison for hit deciding, etc.
    */

    always @(address, readaccess, writeaccess, writedata, cache[index]) begin

        if (readaccess == 1'b1 || writeaccess == 1'b1) begin
            
            // splitting the input address
            #1
            OFFSET = address[1:0];
            index = address[4:2];
            tag = address[7:5];

            valid = cache[index][36];
            dirty = cache[index][35];
            c_tag = cache[index][34:32];

            // dividing the data block into seperate words
            c_data1 = cache[index][7:0];
            c_data2 = cache[index][15:8];
            c_data3 = cache[index][23:16];
            c_data4 = cache[index][31:24];

            // Get data block as it is
            c_data_block = cache[index][31:0];
        end
    end

    // Checking MISS or HIT
    always @(valid, tag, c_tag) begin
        if (valid == 1'b1 && tag[0] == c_tag[0] && tag[1] == c_tag[1] && tag[2] == c_tag[2]) begin // Comparing tag and check the valid bit
            #0.9
            hit = 1'b1;         // HIT
        end

        else begin
            #0.9
            hit = 1'b0;         // MISS
        end
    end

    // Returning the read data according to offset
    always @(c_data1, c_data2, c_data3, c_data4, OFFSET) begin
        #1
        case (OFFSET)
            2'b00:
                readdata = c_data1;
            2'b01:
                readdata = c_data2;
            2'b10:
                readdata = c_data3;
            2'b11:
                readdata = c_data4;
        endcase
    end

    // Resolving the misses and hits
    always @(posedge clock) begin

       /*|||||||||||||||| READ HIT ||||||||||||||||||*/
       if (readaccess && hit) begin
           
           busywait = 1'b0;         // Set busywait to zero
           readaccess = 1'b0;       // Set readaccess to zero
           cache[index][36] = 1'b1; // Set valid bit to 1
       end

        /*|||||||||||||||| WRITE HIT ||||||||||||||||||*/
       else if (writeaccess && hit) begin

           busywait = 0;

           #1
            case (OFFSET)
                2'b00:
                    cache[index][7:0] = writedata;
                2'b01:
                    cache[index][15:8] = writedata;
                2'b10:
                    cache[index][23:16] = writedata;
                2'b11:
                    cache[index][31:24] = writedata;

            endcase

            cache[index][35] = 1'b1;        // Set dirty bit to high becouse values are different from data memory
            cache[index][36] = 1'b1;        // Set valid bit to 1
            writeaccess = 0;                // Set writeaccess to zero after write operation completed 

       end
    end

    
    /* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001, MEM_WRITE = 3'b011, FETCH = 3'b111;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if ((read || write) && !dirty && !hit) begin
                    next_state = MEM_READ;
                end

                else if ((read || write) && dirty && !hit) begin
                    next_state = MEM_WRITE;
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

            MEM_WRITE:
                if (!mem_busywait) begin
                    next_state = MEM_READ;
                end

                else begin
                    next_state = MEM_WRITE;
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
                mem_read = 0;
                mem_write = 0;
                mem_address = 8'dx;
                mem_writedata = 8'dx;
            end
         
            MEM_READ: 
            begin
                mem_read = 1;
                mem_write = 0;
                mem_address = {tag, index};
                mem_writedata = 32'dx;
                busywait = 1;
            end

            MEM_WRITE:
            begin
                mem_read = 0;
                mem_write = 1;
                mem_address = {tag, index};
                mem_writedata = c_data_block;
                busywait = 1;
            end

            FETCH:
            begin
               mem_read = 0;
               mem_write = 0;
               mem_address = 6'bx;
               mem_writedata = 32'bx;
               busywait = 1;

            #1  cache[index][31:0] = memReaddata;	//write a data block to the cache
                cache[index][34:32] = tag;	//tag C_Address[7:5]
                cache[index][35] = 1'b0;	//dirty bit=0 since we are writing a data in cache which is also in memory
                cache[index][36] = 1'b1;	//valid bit

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

    

    initial begin
        $dumpfile("cpu_wavedata.vcd") ;
        
        for (j = 0;j<8 ; j+=1)
            $dumpvars(1,cache[j]) ;
        
    end

endmodule