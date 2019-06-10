// --------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> Cla_substractYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
// --------------------------------------------------------------------
// Cla_substractyright (c) 2019 by UCSD CSE 140L
// --------------------------------------------------------------------
//
// Permission:
//
//   This code for use in UCSD CSE 140L.
//   It is synthesisable for Lattice iCEstick 40HX.  
//
// Disclaimer:
//
//   This Verilog source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  
//
//   git account name: Ailsa98
//
module Lab2_140L (
  input wire Gl_rst,                  // reset signal (active high)
  input wire clk,                     // global clock
  input wire Gl_adder_start,          // r1, r2, OP are ready  
  input wire Gl_subtract,             // subtract (active high)
  // 1:first digit, 2:second digit
  input wire [7:0] Gl_r11,             // 8bit number 1_1
  input wire [7:0] Gl_r12,             // 8bit number 1_2
  input wire [7:0] Gl_r21,             // 8bit number 2_1
  input wire [7:0] Gl_r22,             // 8bit number 2_2
  output wire [7:0] L2_adder_data1,    // 8 bit ascii sum 1
  output wire [7:0] L2_adder_data2,    // 8 bit ascii sum 2
  output wire L2_adder_rdy1,           // pulse1
  output wire L2_adder_rdy2,           // pulse2
  output wire [7:0] L2_led
);

wire c0;  // carry-out bit of last digits
wire c1;
reg rdy2;
wire [3:0] add1;  // temp first digit

// tell the glue logic when addtion is done
sigDelay  sig1(L2_adder_rdy1, Gl_adder_start, clk, Gl_rst);
sigDelay  sig2(L2_adder_rdy2, L2_adder_rdy1, clk, Gl_rst);

// do the 4-bit addition/substraction for the last 4 bits of the input (last byte)
fourBitAdderSubstractor fo2(L2_adder_data2[3:0], c0, Gl_r12[3:0], 
  Gl_r22[3:0], Gl_subtract);

// do the 4-bit addition/substraction for the first 4 bits of the input
fourBitAdderSubstractor fo1(add1, c1, Gl_r11[3:0], 
  Gl_r21[3:0], Gl_subtract);

// modify the first byte based on the carry condition
fourBitAdderSubstractor foc(L2_adder_data1[3:0], L2_led[4], add1, 
  c0 ? 4'b0001 : 4'b0, Gl_subtract);

// change the bits of L2_led to display the result
assign L2_led[3:0] = L2_adder_data1[3:0];


// store the ascii value of the sum
assign L2_adder_data1[7:4] = (L2_led[4] == 0) ? 4'b0011 : 4'b0101;
assign L2_adder_data2[7:4] = (c0 == 0) ? 4'b0011 : 4'b0101;

endmodule


module sigDelay(
  output sigOut,  // is result ready?
  input sigIn,  // are numbers to add ready?
  input clk,  // clock signal
  input rst);  // reset signal

    parameter delayVal = 4;
    reg [15:0] 		      delayReg;


    always @(posedge clk) begin
      if (rst)
        delayReg <= 16'b0;
      else begin
        delayReg <= {delayReg[14:0], sigIn};
      end
    end

    assign sigOut = delayReg[delayVal];
endmodule // sigDelay

//
//  module fullAdder
//  process 1-bit numbers A, B, and Cin from and add them together
//
module fullAdder(
  output  fa_Sum, // sum
  output  fa_Cout,  // carry-out bit
  input fo_A, // augend 
  input fo_B, // addend
  input fo_Cin); // carry-in bit


  assign  fa_Sum = fo_A ^ fo_B ^ fo_Cin;  // sum = A xor B xor Cin
  // carry-out = 1 if there are two or more 1s in the input
  assign  fa_Cout = fo_A & fo_B | fo_A & fo_Cin | fo_B & fo_Cin;

endmodule

//
//  module fourBitAdderSubstractor
//  Process two 4-bit numbers and add/substract them using fullAdder
//
module fourBitAdderSubstractor(
   output [3:0] fo_Sum, // sum/difference
   output 	fo_C,  // carryout/borrow status
   input [3:0]  la_A, // 4-bit augend/minuend
   input [3:0]  la_B, // 4-bit addend/subtrahend
   input  la_substract);  // subtraction or addition or carry in bit
   
   // wires connect fullAdders
   wire C0; // Cout of fa0, Cin of fa1.
   wire C1; // Cout of fa1, Cin of fa2.
   wire	C2; // Cout of fa2, Cin of fa3.
   wire	C3; // Cout of fa2, used to generate final carry/borrrow
   
   // wires store the addend or substrahend
   wire B0; //  B[0] xor la_substract
   wire	B1; //  B[1] xor la_substract
   wire B2; //  B[2] xor la_substract
   wire	B3; //  B[3] xor la_substract
   
   // modify the addend to its one's complement used in substraction
   assign B0 = la_B[0] ^ la_substract;
   assign B1 = la_B[1] ^ la_substract;
   assign B2 = la_B[2] ^ la_substract;
   assign B3 = la_B[3] ^ la_substract;

   // call fullAdder for each bit (add 1 to form 2's complement in substraction)
   fullAdder fa0(fo_Sum[0], C0, la_A[0], B0, la_substract); // Least significant bit
   fullAdder fa1(fo_Sum[1], C1, la_A[1], B1, C0);
   fullAdder fa2(fo_Sum[2], C2, la_A[2], B2, C1);
   fullAdder fa3(fo_Sum[3], C3, la_A[3], B3, C2); // Most significant bit

   // carry or borrow if the final Cout bit is 1 in addition 
   // or 0 in substraction
   assign fo_C = C3 ^ la_substract;

endmodule
