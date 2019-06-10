
// --------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
// --------------------------------------------------------------------
// Copyright (c) 2019 by UCSD CSE 140L
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
// -------------------------------------------------------------------- //      
//
//                     Lih-Feng Tsaur
//                     Bryan Chin
//                     UCSD CSE Department
//                     9500 Gilman Dr, La Jolla, CA 92093
//                     U.S.A
//
// --------------------------------------------------------------------
//
// bcd2segment
//
// convert binary coded decimal to seven segment display
//
//                        aaa
//                       f   b 
//                       f   b
//                       f   b				
//                        ggg
//                       e   c
//                       e   c
//                       e   c
//                        ddd 
//
// segment[0] - a     segment[3] - d    segment[6] - g
// segment[1] - b     segment[4] - e
// segment[2] - c     segment[5] - f
//
module bcd2segment (
		  output wire [6:0] segment,  // 7 drivers for segment
		  input  wire [3:0] num,       // number to convert
		  input wire enable          // if 1, drive display, else blank
		  );


   wire    zero = (~|num);
   wire	   one = (~|(num[3:0]^4'b0001));
   wire	   two = (~|(num[3:0]^4'b0010)); 
   wire	   three = (~|(num[3:0]^4'b0011));
   wire	   four = (~|(num[3:0]^4'b0100));
   wire	   five = (~|(num[3:0]^4'b0101));
   wire	   six = (~|(num[3:0]^4'b0110));
   wire	   seven = (~|(num[3:0]^4'b0111));
   wire    eight = (~|(num[3:0]^4'b1000));
   wire	   nine = (~|(num[3:0]^4'b1001));
   wire	   ten = (~|(num[3:0]^4'b1010));
   wire	   eleven = (~|(num[3:0]^4'b1011));
   wire    twelve = (~|(num[3:0]^4'b1100));
   wire	   thirteen = (~|(num[3:0]^4'b1101));
   wire    fourteen = (~|(num[3:0]^4'b1110));
   wire	   P = (~|(num[3:0]^4'b1111));



   wire [6:0] segmentUQ;
   
   // a
   assign segmentUQ[0] =  (
		       zero | two | three | five | six | seven | eight | nine | ten |
		       twelve | fourteen | P);
   // b
   assign segmentUQ[1] = (
		       zero | one | two | three | four | seven |
		       eight | nine | ten | thirteen | P);
   // c
   assign segmentUQ[2] = (
		      zero | one | three | four | five | six | seven |
		      eight | nine | ten | eleven | thirteen) ;
   
   // d
   assign segmentUQ[3] = (
		      zero | two | three | five | six |
		      eight | eleven | twelve | thirteen | fourteen) ;

   // e
   assign segmentUQ[4] = (  
          zero | two | six | eight | ten | eleven | twelve |
          thirteen | fourteen | P);
   
   // f
   assign segmentUQ[5] = ( 
          zero | four | five | six | eight | nine |
          ten | eleven | twelve | fourteen | P);

   // g
   assign segmentUQ[6] = (
          two | three | four | five | six | eight | nine | ten |
		      eleven | thirteen | fourteen | P);

   assign segment = {7{enable}} & segmentUQ;
   
endmodule
