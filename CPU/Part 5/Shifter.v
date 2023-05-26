module SHIFTER(VALUE, AMOUNT, RESULT, SHIFT_TYPE);          // Module for Left and Right Shifting

    input [7:0] VALUE;          // Number for shifting
    input [7:0] AMOUNT;         // Amount of shifting
    input SHIFT_TYPE;           // Select left shift or right shift
    output [7:0] RESULT;        // Return the result

    





endmodule


module LSU(IN1, IN2, S, OUT);    // Unit module for left shifting

    input IN1, IN2, S;          // Inputs
    output OUT;                 // Output of module

    wire OR1, OR2;              // Wires for OR gate

    and A1(OR1, ~S, IN1);       // AND gate with selecting
    and A1(OR2, S, IN2);        // AND gate with selecting

    or OR1 (OUT, OR1, OR2);      // Producing output for 2 bit

endmodule



module LEFT_SHIFT(IN, SHIFT, OUT);          // Module for left shift

    input [7:0] IN;                         // Number for shifting
    input [3:0] SHIFT;                      // Number of bits for shifting
    output reg signed [7:0] OUT;                       // Output after shifting

    // Here we are dealing with 8 bit signed numbers. Then most significant bit for sign of number
    // So, We only want shifting 7 bit number to left when considering sign

    reg S1, S2, S3, S4;                        // for shifting (here only we have to shift 7 bits, So 3 wires required)

    initial begin                          // Set shifting amount to zero when begin
        S1 = 1'B0;
        S2 = 1'B0;
        S3 = 1'B0;
        S4 = 1'B0;
    end

    wire X1, X2, X3, X4, X5, X6, X7, X8;       // get output from S1 shifting
    wire Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8;       // get output from S2 shifting
    wire Z1, Z2, Z3, Z4, Z5, Z6, Z7, Z8;       // get output from S3 Shifting
    wire F1, F2, F3, F4, F5, F6, F7, F8;       // get output from S4 shifting

    always@(IN, SHIFT) begin                   // Assign Shifting values
        S1 = SHIFT[0];
        S2 = SHIFT[1];
        S3 = SHIFT[2];
        S4 = SHIFT[3];
    end

    // Shifting from S1

    LSU L1(IN[0], 1'B0, S1, X1);
    LSU L2(IN[1], IN[0], S1, X2);
    LSU L3(IN[2], IN[1], S1, X3);
    LSU L4(IN[3], IN[2], S1, X4);
    LSU L5(IN[4], IN[3], S1, X5);
    LSU L6(IN[5], IN[4], S1, X6);
    LSU L7(IN[6], IN[5], S1, X7);
    LSU L8(IN[7], IN[6], S1, X8);


    // Shifting form S2

    LSU L9(X1, 1'B0, S2, Y1);
    LSU L10(X2, 1'B0, S2, Y2);
    LSU L11(X3, X1, S2, Y3);
    LSU L12(X4, X2, S2, Y4);
    LSU L13(X5, X3, S2, Y5);
    LSU L14(X6, X4, S2, Y6);
    LSU L15(X7, X5, S2, Y7);
    LSU L16(X8, X6, S2, Y8);

    // Shifting from S3

    LSU L17(Y1, 1'B0, S3, Z1);
    LSU L18(Y2, 1'B0, S3, Z2);
    LSU L19(Y3, 1'B0, S3, Z3);
    LSU L20(Y4, 1'B0, S3, Z4);
    LSU L21(Y5, Y1, S3, Z5);
    LSU L22(Y6, Y2, S3, Z6);
    LSU L23(Y7, Y3, S3, Z7);
    LSU L24(Y8, Y4, S3, Z8);

    // Shifting from S4

    LSU L25(Z1, 1'B0, S4, F1);
    LSU L26(Z2, 1'B0, S4, F2);
    LSU L27(Z3, 1'B0, S4, F3);
    LSU L28(Z4, 1'B0, S4, F4);
    LSU L29(Z5, 1'B0, S4, F5);
    LSU L30(Z6, 1'B0, S4, F6);
    LSU L31(Z7, 1'B0, S4, F7);
    LSU L32(Z8, 1'B0, S4, F8);


    always@(F1, F2, F3, F4, F5, F6, F7, F8) begin
        OUT[0] = F1;
        OUT[1] = F2;
        OUT[2] = F3;
        OUT[3] = F4;
        OUT[4] = F5;
        OUT[5] = F6;
        OUT[6] = F7;
        OUT[7] = F8;
    end


endmodule