// CO224 Lab 05
// Part 01 - ALU
// Group No - 07

`timescale 1ns/100ps

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
        
        //default:
            //assign out = 8'bx;
        endcase    
    end
endmodule

// This is the ALU module to instantiate above module and select
module ALU(DATA1, DATA2, RESULT, SELECT, ZERO);
    input [7:0] DATA1, DATA2;       // Input data1 and data2 as 8bit bus
    input [2:0] SELECT;             // Input select data as 3bit bus
    output [7:0] RESULT;            // Output the result as 8ibt bus
    output reg ZERO;
    
    wire [7:0] temp0, temp1, temp2, temp3, temp4, temp5, temp6, temp7;  // Decalring temp values to get outputs from modules

    FORWARD FORWARD1(DATA2, temp0);       // Instantiate FORWARD module
    ADD ADD1(DATA1, DATA2, temp1);        // Instantiate ADD module
    AND AND1(DATA1, DATA2, temp2);        // Instantiate AND  module
    OR OR1(DATA1, DATA2, temp3);          // Instantiate OR module

    Mux_8x1 Mux(RESULT, temp0, temp1, temp2, temp3, temp4, temp5, temp6, temp7, SELECT);    // Instantiate Mux_8x1 module to select the out that we want

    /* initial begin
        ZERO = 1'b0;
    end */

    always @(DATA1, DATA2, SELECT, temp1) begin
        if (temp1 == 8'b00000000) begin
            ZERO = 1'b1;
        end 

        else begin
            ZERO = 1'b0;
        end
    end

    

endmodule
