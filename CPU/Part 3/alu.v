// CO224 Lab 05
// Part 01 - ALU
// Group No - 07


// This module simply send the operand value to its output
module FORWARD(data, result);
    input [7:0] data;       // Getting data to operate
    output [7:0] result;    // Output the operated data

    assign #1 result = data;   // Assign the data value to result
endmodule

// This module add data1 and data2 and give as result
module ADD(data1, data2, result);
    input [7:0] data1, data2;   // Getting data1 and data2 to operate
    output signed[7:0] result;        // Output the operated data

    assign #2 result = data1 + data2;  // add data1 and data2, then assign to result
endmodule

// This module operate bitwise AND on data1 with data2
module AND(data1, data2, result);
    input [7:0] data1, data2;       // Getting data1 and data2 to operate
    output [7:0] result;            // Output the operated data

    assign #1 result = data1 & data2;  // assign result to bitwise AND data1 with data2 
endmodule

// This module operate bitwise OR on data1 with data2
module OR(data1, data2, result);    
    input [7:0] data1, data2;       // Getting data1 and data2 to operate
    output [7:0] result;            // Output the operated data

    assign #1 result = data1 | data2;
endmodule

// This is the mux module for Selct
module Mux_8x1(out, D0, D1, D2, D3, D4, D5, D6, D7, Select);
    input [7:0] D0, D1, D2, D3, D4, D5, D6, D7;
    input [2:0] Select;
    output reg[7:0] out;

    always @(D0, D1, D2, D3, D4, D5, D6, D7, Select)
    begin
        case (Select)
        3'b000:
            assign out = D0;             
        3'b001:
            assign out = D1;           
        3'b010:
            assign out = D2;           
        3'b011:
            assign out = D3;
        3'b100:
            assign out = D4;
        3'b101:
            assign out = D5;
        3'b110:
            assign out = D6;
        3'b111:
            assign out = D7;
        
        default:
            assign out = 8'bx;
        endcase    
    end
endmodule

// This is the ALU module to instantiate above module and select
module ALU(DATA1, DATA2, RESULT, SELECT);
    input [7:0] DATA1, DATA2;       // Input data1 and data2 as 8bit bus
    input [2:0] SELECT;             // Input select data as 3bit bus
    output [7:0] RESULT;            // Output the result as 8ibt bus
    
    wire [7:0] temp0, temp1, temp2, temp3, temp4, temp5, temp6, temp7;  // Decalring temp values to get outputs from modules

    FORWARD FORWARD1(DATA2, temp0);       // Instantiate FORWARD module
    ADD ADD1(DATA1, DATA2, temp1);        // Instantiate ADD module
    AND AND1(DATA1, DATA2, temp2);        // Instantiate AND  module
    OR OR1(DATA1, DATA2, temp3);          // Instantiate OR module

    Mux_8x1 Mux(RESULT, temp0, temp1, temp2, temp3, temp4, temp5, temp6, temp7, SELECT);    // Instantiate Mux_8x1 module to select the out that we want

endmodule

// Testbench for ALU
/*module testbench;
    reg [7:0] Data1, Data2;         // 8 bit Data1 Data2 registers to input data
    reg [2:0] Select;               // 3 bit Select register as selector to the mux in ALU
    wire [7:0] Result;              // 8 bit Result wire to output the result

    ALU ALU1(Data1, Data2, Result, Select);     // Instantiation ALU module

    initial begin
        $monitor("Data1 = %b(%d), Data2 = %b(%d), Select = %b, Result = %b(%d)", Data1, Data1, Data2, Data2, Select, Result, Result);
        $dumpfile("alu2.vcd");
        $dumpvars(0, testbench);

        Select = 3'b000;
        Data1 = 8'b01010101;    // 85
        Data2 = 8'b00001111;    // 15

        
        #3 Select = 3'b000;     // Forward
        #3 Select = 3'b001;     // ADD
        #3 Select = 3'b010;     // AND
        #3 Select = 3'b011;     // OR
        
    end

endmodule
*/