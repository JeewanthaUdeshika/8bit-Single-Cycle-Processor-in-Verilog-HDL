`include "MyShifter.v"

module TestBench;

    reg signed [7:0] IN;
    reg [3:0] SHIFT;
    wire signed[7:0] OUT;

    //LSU LSU1(IN1, IN2, S, OUT);

    rightShifter lf1(OUT, IN, SHIFT);

    

    initial begin
        //$monitor("IN1 = %B      IN2 = %B      S= %B       OUT = %B", IN1, IN2, S, OUT);

        $monitor("IN = %b (%d)      SHIFT = %b (%d)         OUT = %B (%d)", IN, IN, SHIFT, SHIFT, OUT, OUT);

        IN = 8'b11010101;

        #1 SHIFT = 4'B0000;

        #1 SHIFT = 4'B0001;

        #1 SHIFT = 4'B0010;

        #1 SHIFT = 4'B0011;

        #1 SHIFT = 4'B0100;

        #1 SHIFT = 4'B0101;

        #1 SHIFT = 4'B0110;

        #1 SHIFT = 4'B0111;



        /*#1
        SHIFT = 3'B010;

        #1
        SHIFT = 3'B011;

        #1
        SHIFT = 3'B111;*/



        /*IN1 = 0;
        IN2 = 0;
        S =0;

        #1
        IN1 = 1;
        IN2 = 0;
        S =0;

        #1
        IN1 = 1;
        IN2 = 0;
        S =1;*/


    end



endmodule
