// Logical left Shifter module for cpu
// Lab05 - Part05

// This module shift one bit to left
module LSU (IN0, IN1, S, OUT);
    input IN0, IN1, S;
    output OUT;

    wire and1, and2;

    and AND1(and1, ~S, IN0);
    and AND2(and2, S, IN1);
    or OR1(OUT, and1, and2);

endmodule


// This module do logical shift left to given input by given number
module leftShifter(OUT, IN, SHIFT);
    input [7:0] IN;
    input [3:0] SHIFT;
    output [7:0] OUT;

    wire a0, a1, a2, a3, a4, a5, a6, a7;
    wire b0, b1, b2, b3, b4, b5, b6, b7;
    wire c0, c1, c2, c3, c4, c5, c6, c7;

    // Shifting for first bit
    LSU A7(IN[7], IN[6], SHIFT[0], a7);
    LSU A6(IN[6], IN[5], SHIFT[0], a6);
    LSU A5(IN[5], IN[4], SHIFT[0], a5);
    LSU A4(IN[4], IN[3], SHIFT[0], a4);
    LSU A3(IN[3], IN[2], SHIFT[0], a3);
    LSU A2(IN[2], IN[1], SHIFT[0], a2);
    LSU A1(IN[1], IN[0], SHIFT[0], a1);
    LSU A0(IN[0], 1'b0, SHIFT[0], a0);

    // Shifting for second bit
    LSU B7(a7, a5, SHIFT[1], b7);
    LSU B6(a6, a4, SHIFT[1], b6);
    LSU B5(a5, a3, SHIFT[1], b5);
    LSU B4(a4, a2, SHIFT[1], b4);
    LSU B3(a3, a1, SHIFT[1], b3);
    LSU B2(a2, a0, SHIFT[1], b2);
    LSU B1(a1, 1'b0, SHIFT[1], b1);
    LSU B0(a0, 1'b0, SHIFT[1], b0);

    // Shifting for third bit
    LSU C7(b7, b3, SHIFT[2], c7);
    LSU C6(b6, b2, SHIFT[2], c6);
    LSU C5(b5, b1, SHIFT[2], c5);
    LSU C4(b4, b0, SHIFT[2], c4);
    LSU C3(b3, 1'b0, SHIFT[2], c3);
    LSU C2(b2, 1'b0, SHIFT[2], c2);
    LSU C1(b1, 1'b0, SHIFT[2], c1);
    LSU C0(b0, 1'b0, SHIFT[2], c0);

    // Shifting for fourth bit
    LSU D7(c7, 1'b0, SHIFT[3], OUT[7]);
    LSU D6(c6, 1'b0, SHIFT[3], OUT[6]);
    LSU D5(c5, 1'b0, SHIFT[3], OUT[5]);
    LSU D4(c4, 1'b0, SHIFT[3], OUT[4]);
    LSU D3(c3, 1'b0, SHIFT[3], OUT[3]);
    LSU D2(c2, 1'b0, SHIFT[3], OUT[2]);
    LSU D1(c1, 1'b0, SHIFT[3], OUT[1]);
    LSU D0(c0, 1'b0, SHIFT[3], OUT[0]);

endmodule

// This module do logical shift left to given input by given number
module rightShifter(OUT, IN, SHIFT);
    input [7:0] IN;
    input [3:0] SHIFT;
    output [7:0] OUT;

    wire a0, a1, a2, a3, a4, a5, a6, a7;
    wire b0, b1, b2, b3, b4, b5, b6, b7;
    wire c0, c1, c2, c3, c4, c5, c6, c7;
    
    // Shifting for firstbit
    LSU A7(IN[7], 1'b0, SHIFT[0], a0);
    LSU A6(IN[6], IN[7], SHIFT[0], a1);
    LSU A5(IN[5], IN[6], SHIFT[0], a2);
    LSU A4(IN[4], IN[5], SHIFT[0], a3);
    LSU A3(IN[3], IN[4], SHIFT[0], a4);
    LSU A2(IN[2], IN[3], SHIFT[0], a5);
    LSU A1(IN[1], IN[2], SHIFT[0], a6);
    LSU A0(IN[0], IN[1], SHIFT[0], a7);

    // Shifting for Second bit
    LSU B7(a0, 1'b0, SHIFT[1], b0);
    LSU B6(a1, 1'b0, SHIFT[1], b1);
    LSU B5(a2, a0, SHIFT[1], b2);
    LSU B4(a3, a1, SHIFT[1], b3);
    LSU B3(a4, a2, SHIFT[1], b4);
    LSU B2(a5, a3, SHIFT[1], b5);
    LSU B1(a6, a4, SHIFT[1], b6);
    LSU B0(a7, a5, SHIFT[1], b7);

    // Shifting for Third bit
    LSU C7(b0, 1'b0, SHIFT[2], c0);
    LSU C6(b1, 1'b0, SHIFT[2], c1);
    LSU C5(b2, 1'b0, SHIFT[2], c2);
    LSU C4(b3, 1'b0, SHIFT[2], c3);
    LSU C3(b4, b0, SHIFT[2], c4);
    LSU C2(b5, b1, SHIFT[2], c5);
    LSU C1(b6, b2, SHIFT[2], c6);
    LSU C0(b7, b3, SHIFT[2], c7);

    // Shifting for Fourth bit
    LSU D7(c0, 1'b0, SHIFT[3], OUT[7]);
    LSU D6(c1, 1'b0, SHIFT[3], OUT[6]);
    LSU D5(c2, 1'b0, SHIFT[3], OUT[5]);
    LSU D4(c3, 1'b0, SHIFT[3], OUT[4]);
    LSU D3(c4, 1'b0, SHIFT[3], OUT[3]);
    LSU D2(c5, 1'b0, SHIFT[3], OUT[2]);
    LSU D1(c6, 1'b0, SHIFT[3], OUT[1]);
    LSU D0(c7, 1'b0, SHIFT[3], OUT[0]);

 
endmodule

// This module is main module that can shift lleft and right. This is added to the ALU module
module logicalShift (OUT, IN, shiftVal, shiftSignal);
    input[7:0] IN;
    input[7:0] shiftVal;
    input shiftSignal;
    output reg[7:0] OUT;
    wire[7:0] temp1, temp2;

    // Initiating Leftshifter and Rightsifter
    leftShifter LS1(temp1, IN, shiftVal[3:0]);
    rightShifter RS2(temp2, IN, shiftVal[3:0]);


    always @(IN, shiftVal, shiftSignal) begin
        if (shiftSignal == 1'b1) begin
           #1 OUT = temp1;
        end

        else if (shiftSignal == 1'b0) begin
           #1 OUT = temp2;
        end
    end

endmodule