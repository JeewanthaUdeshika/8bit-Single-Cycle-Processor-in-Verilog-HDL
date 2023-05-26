/*
CO224 - Lab06 Part 02
Group 07

Cache controller module for CPU using FSM
*/

module data_memory(
	clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
    busywait
);
input           clock;
input           reset;
input           read;
input           write;
input[7:0]      address;
input[7:0]      writedata;
output reg [7:0]readdata;
output reg      busywait;
integer j;

//Declare memory array 256x8-bits 
reg [7:0] memory_array [255:0];

//Detecting an incoming memory access
reg readaccess, writeaccess;
always @(read, write)
begin
	busywait = (read || write)? 1 : 0;
	readaccess = (read && !write)? 1 : 0;
	writeaccess = (!read && write)? 1 : 0;
end

//Reading & writing
always @(posedge clock)
begin
    if(readaccess)
    begin
        readdata = #40 memory_array[address];
        busywait = 0;
		readaccess = 0;
    end
    if(writeaccess)
	begin
        memory_array[address] = #40 writedata;
        busywait = 0;
		writeaccess = 0;
    end
end

integer i;

//Reset memory
always @(posedge reset)
begin
    if (reset)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] = 0;
        
        busywait = 0;
		readaccess = 0;
		writeaccess = 0;
    end
end

initial begin
        $dumpfile("cpu_wavedata.vcd") ;
        
        for (j = 0;j<12 ; j+=1)
            $dumpvars(1,memory_array[j]) ;
        
    end

endmodule