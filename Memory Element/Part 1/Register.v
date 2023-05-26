// CO224 Lab 5
// Part 2 V2

// This is the register module for write and read 8 8bit regs.
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
    input [7:0] IN;                                      // Data that to be written
    input [2:0] INADDRESS, OUT1ADDRESS, OUT2ADDRESS;     // Addresses(indexes) to be written and to be out
    input WRITE, CLK, RESET;                            // Writeenable, CLOCK and Reset as inputs
    output [7:0] OUT1, OUT2;                             // Data that should be outputs

    reg[7:0] registers[7:0];                            // Making array of eight 8bit registers
    reg[7:0] OUT1, OUT2;                                // Declaring OUT1 and OUT2 as regs
    integer i, j;                                          // Declaring i as integer to for loop 

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

    initial begin
        $dumpfile("cpu_wavedata.vcd") ;
        
        for (j = 0;j<8 ; j+=1)
            $dumpvars(1,registers[j]) ;
        
    end

    initial
	begin
		#5;
		$display("\n\t\t\t___________________________________________________");
		$display("\n\t\t\t CHANGE OF REGISTER CONTENT STARTING FROM TIME #5");
		$display("\n\t\t\t___________________________________________________\n");
		$display("\t\ttime\treg0\treg1\treg2\treg3\treg4\treg5\treg6\treg7");
		$display("\t\t____________________________________________________________________");
		$monitor($time, "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",registers[0],registers[1],registers[2],registers[3],registers[4],registers[5],registers[6],registers[7]);
	end

    
endmodule