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
  /*
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
*/

  input wire   rst, // reset signal (active high)
  input wire   clk,
  input        bu_rx_data_rdy, // data from the uart is ready
  input [7:0]  bu_rx_data, // data from the uart
  output       Gl_tx_data_rdy, // data ready to be sent to UART
  output [7:0] Gl_tx_data, // data to be sent to UART
  output [4:0] L2_led
);

wire [3:0] c0;  // carry-out bit of last digits
wire c1;
reg rdy2;
wire [3:0] add1;  // temp first digit
wire [7:0] L2_adder_data1;
wire [7:0] L2_adder_data2;      
wire L2_adder_rdy;

// tell the glue logic when addtion is done
sigDelay  sig(L2_adder_rdy, Gl_adder_start, clk, Gl_rst);

// do the 4-bit addition/substraction for the last 4 bits of the input (last byte)
fourBitAdderSubstractor fo2(L2_adder_data2[3:0], c0[0], Gl_r12[3:0], 
  Gl_r22[3:0], Gl_subtract);

// do the 4-bit addition/substraction for the first 4 bits of the input
fourBitAdderSubstractor fo1(add1, c1, Gl_r11[3:0], 
  Gl_r21[3:0], Gl_subtract);

// modify the first byte based on the carry condition
fourBitAdderSubstractor foc(L2_adder_data1[3:0], L2_led[4], add1, 
  c0, Gl_subtract);

// change the bits of L2_led to display the result
assign L2_led[3:0] = L2_adder_data1[3:0];


// store the ascii value of the sum
assign L2_adder_data1[7:4] = (L2_led[4] == 0) ? 4'b0011 : 4'b0101;
assign L2_adder_data2[7:4] = (c0 == 0) ? 4'b0011 : 4'b0101;

Glue_Lab2 Glue_Lab2 (
  .Gl_r11(Gl_r11),
  .Gl_r12(Gl_r12),
  .Gl_r21(Gl_r21),
  .Gl_r22(Gl_r22),
  .Gl_subtract(Gl_subtract),
  .Gl_adder_start(Gl_adder_start),
  .bu_rx_data(bu_rx_data),
  .bu_rx_data_rdy(bu_rx_data_rdy),
  .bu_tx_busy(bu_tx_busy),
  .L2_adder_data1(L2_adder_data1), 
  .L2_adder_data2(L2_adder_data2),  // fix me - should get 8 bits from lab2
  .L2_adder_rdy(L2_adder_rdy),
  .Gl_tx_data(Gl_tx_data),
  .Gl_tx_data_rdy(Gl_tx_data_rdy),

  .Gl_rst(Gl_rst),
  .L2_led(L2_led),
  .led(led),
  .tb_sim_rst(tb_sim_rst),
  .clk(clk));

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

// _Lab2
// glue logic for lab 2
//
// we capture 4 8 bit values
// Gl_r11, Gl_r12, Gl_r21, and Gl_r22
// we capture an operation (either add or subtract)
// finally we generate a start pulse - Gl_adder_start which lasts for one cycle
//
module Glue_Lab2(
  output reg [7:0]  Gl_r11,
  output reg [7:0]  Gl_r12,
  output reg [7:0]  Gl_r21,
  output reg [7:0]  Gl_r22,
  output wire 	   Gl_subtract,
  output wire 	   Gl_adder_start,

  // from uart
  input [7:0] 	   bu_rx_data,
  input 		   bu_rx_data_rdy,
  input             bu_tx_busy, 		   

  // from DUT
  input [7:0] 	   L2_adder_data1,
  input 		   L2_adder_rdy,
  input [7:0] 	   L2_adder_data2,
  // to uart
  output reg [7:0]  Gl_tx_data,
  output reg 	   Gl_tx_data_rdy,


  output reg 	   Gl_rst, // internal reset

  // LED interface
  input [7:0] 	   L2_led, // from DUT
  output wire [4:0] led,
  input 		   tb_sim_rst,
  input 		   clk
);



wire i_rst_char_detected;      // reset character is 0x1B  (ESC)
wire is_hexplus;               // 0-9:;<=>? abcdef
wire is_pn;			  // plus, minus

decodeKeys de0 (
  .de_esc(i_rst_char_detected),
  .de_hexplus(is_hexplus),
  .de_pn(is_pn),
  .charData(bu_rx_data),
  .charDataValid(bu_rx_data_rdy)	   
);

wire 			   enR11;     // enable first byte of R1 register
wire 			   enR12;     // enable second byte of R1 register
wire 			   enR21;     // enable first byte of R2 register
wire 			   enR22;     // enable second byte of R2 register
wire 			   enOp;     // enable OP register

gl_fsm gl_fsm(
  .enR11(enR11),
  .enR12(enR12),
  .enR21(enR21),
  .enR22(enR22),
  .enOp(enOp),
  .Gl_adder_start(Gl_adder_start),
  .bu_tx_busy(bu_tx_busy),
  .validInput(is_hexplus),
  .validOp(is_pn),
  .clk(clk),
  .Gl_rst(Gl_rst)
  `ifdef HW
  `else
    ,.tb_sim_rst(tb_sim_rst)
  `endif
  );



  reg [7:0] 		   Gl_Op;     // save glue op


  // -----------------------------
  // datapath - save operands
  //
  //
  wire dp_rst = Gl_rst | tb_sim_rst;

  always @(posedge clk) begin
    if (enR11 | dp_rst)
      Gl_r11 <= dp_rst ? 8'b0 : bu_rx_data;
    else
      Gl_r11 <= Gl_r11;

    if (enR12 | dp_rst)
      Gl_r12 <= dp_rst ? 8'b0 : bu_rx_data;
    else
      Gl_r12 <= Gl_r12;

    if (enR21 | dp_rst)
      Gl_r21 <= dp_rst ? 8'b0 : bu_rx_data;
    else
      Gl_r21 <= Gl_r21;

    if (enR22 | dp_rst )
      Gl_r22 <= dp_rst ? 8'b0 : bu_rx_data;
    else
      Gl_r22 <= Gl_r22;

    if (enOp | dp_rst )
      Gl_Op <= dp_rst ? 8'b0 : bu_rx_data;
    else
      Gl_Op <= Gl_Op;
  end

  // decode Gl_Op (+ or -)
  // 
  assign Gl_subtract = (Gl_Op == "-") ? 1 : 0;

  sigDelay  sig(L2_adder_rdy2, Gl_adder_rdy, clk, Gl_rst);

  //
  // transmit uart
  // echo input characters and output from adder
  //
  always @(posedge clk) begin
    Gl_tx_data <= is_hexplus ? bu_rx_data : L2_adder_rdy ? L2_adder_data1 : L2_adder_data2;
    Gl_tx_data_rdy <= is_hexplus | L2_adder_rdy | L2_adder_rdy2;
  end

  //
  // reset generator
  //    
  reg [4:0] reset_count;
  always @(posedge clk) begin
    if (tb_sim_rst) begin
      reset_count <= 5'b0000;
    end
    else begin
      if (i_rst_char_detected) begin
        reset_count <= 5'b0000;
        Gl_rst <= 1;
      end
      else begin
        if (~reset_count[4]) begin
          reset_count <= reset_count + 1;
          Gl_rst <= 1;
        end
        else begin
          reset_count <= reset_count;
          Gl_rst <= 0;
        end
      end
    end // else: !if(tb_sim_rst)
  end // always @ (posedge clk_in)


  endmodule

