// CO224 Lab 5
// Part 2 V2

// This is the register module for write and read 8 8bit regs.
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
    input[7:0] IN;                                      // Data that to be written
    input[2:0] INADDRESS, OUT1ADDRESS, OUT2ADDRESS;     // Addresses(indexes) to be written and to be out
    input WRITE, CLK, RESET;                            // Writeenable, CLOCK and Reset as inputs
    output[7:0] OUT1, OUT2;                             // Data that should be outputs

    reg[7:0] registers[7:0];                            // Making array of eight 8bit registers
    reg[7:0] OUT1, OUT2;                                // Declaring OUT1 and OUT2 as regs
    integer i;                                          // Declaring i as integer to for loop 

    // Always block to read reg values 
    always @(                                           // Adding all the registers seperatly because OUT1 and OUT2 should be updated for every register changing
        OUT1ADDRESS,                                    
        OUT2ADDRESS,
        registers[0],
        registers[1],
        registers[2],
        registers[3],
        registers[4],
        registers[5],
        registers[6],
        registers[7]
    ) begin
        #2
        OUT1 <= registers[OUT1ADDRESS];                 // Assign register value of given address to OUT1 and OUT2 with non blocking assigning 
        OUT2 <= registers[OUT2ADDRESS]; 
    end
    

    always @(posedge CLK) begin                         // Write and Reset should be happen in positive edges in clock signal
        if(RESET)                                       // If reset is high, assign zero to all registers in register array
        begin
            #1
            for (i=0; i<8; i=i+1)
            begin
                registers[i] <= 0;
            end
        end

        else begin                                      // Else if write is high, write given data in given address's register
            if(WRITE)
            begin
                #1 registers[INADDRESS] <= IN;
            end
        end
    end
    
endmodule

module reg_file_tb;
    
    reg [7:0] WRITEDATA;
    reg [2:0] WRITEREG, READREG1, READREG2;
    reg CLK, RESET, WRITEENABLE; 
    wire [7:0] REGOUT1, REGOUT2;
    
    reg_file myregfile(WRITEDATA, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);
       
    initial
    begin
        CLK = 1'b1;
        
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("reg_file_wavedata.vcd");
		$dumpvars(0, reg_file_tb);
        
        // assign values with time to input signals to see output 
        RESET = 1'b0;
        WRITEENABLE = 1'b0;
        
        #4
        RESET = 1'b1;
        READREG1 = 3'd0;
        READREG2 = 3'd4;
        
        #6
        RESET = 1'b0;
        
        #2
        WRITEREG = 3'd2;
        WRITEDATA = 8'd95;
        WRITEENABLE = 1'b1;
        
        #7
        WRITEENABLE = 1'b0;
        
        #1
        READREG1 = 3'd2;
        
        #7
        WRITEREG = 3'd1;
        WRITEDATA = 8'd28;
        WRITEENABLE = 1'b1;
        READREG1 = 3'd1;
        
        #8
        WRITEENABLE = 1'b0;
        
        #8
        WRITEREG = 3'd4;
        WRITEDATA = 8'd6;
        WRITEENABLE = 1'b1;
        
        #8
        WRITEDATA = 8'd15;
        WRITEENABLE = 1'b1;
        
        #10
        WRITEENABLE = 1'b0;
        
        #6
        WRITEREG = -3'd1;
        WRITEDATA = 8'd50;
        WRITEENABLE = 1'b1;
        
        #5
        WRITEENABLE = 1'b0;
        
        #10
        $finish;
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule