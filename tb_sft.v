
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
  //                     Lih-Feng Tsaur
  //                     Bryan Chin
  //                     UCSD CSE Department
  //                     9500 Gilman Dr, La Jolla, CA 92093
  //                     U.S.A
  //
  // --------------------------------------------------------------------

  //
  // software testbench for simulation
  //
  `define MAXMSG 256

  module tb_sft(
    output reg   tb_sim_rst,      // soft reset
    output reg   clk12m,          // 12 mhz clock
    input [7:0]  L3_tx_data,      // uart data from (alarm time and status)
    input 	   L3_tx_data_rdy,  // 

    output reg [7:0] tb_rx_data,  // uart data to the DUT
    output reg   tb_rx_data_rdy,  //

    input [4:0]  leds,
    input [6:0] segment1,
    input [6:0] segment2,
    input [6:0] segment3,
    input [6:0] segment4
  );

  //
  // task displayLED
  //
  // emulate a 4 digit seven segment display
  //
  //
  task displayLED(
    input [6:0] segment1,
    input [6:0] segment2,
    input [6:0] segment3,
    input [6:0] segment4
  );
  begin
    // a 
    $display ( "%s %s   %s %s",
      segment4[0] ? " === " : " --- ",
      segment3[0] ? " === " : " --- ",
      segment2[0] ? " === " : " --- ",
      segment1[0] ? " === " : " --- ");
    // f, b
    $display ( "%s   %s %s   %s   %s   %s %s   %s",
      segment4[5] ? "#" : "-", segment4[1] ? "#" : "-",
      segment3[5] ? "#" : "-", segment3[1] ? "#" : "-",
      segment2[5] ? "#" : "-", segment2[1] ? "#" : "-",
      segment1[5] ? "#" : "-", segment1[1] ? "#" : "-");
    $display ( "%s   %s %s   %s   %s   %s %s   %s",
      segment4[5] ? "#" : "-", segment4[1] ? "#" : "-",
      segment3[5] ? "#" : "-", segment3[1] ? "#" : "-",
      segment2[5] ? "#" : "-", segment2[1] ? "#" : "-",
      segment1[5] ? "#" : "-", segment1[1] ? "#" : "-");
    $display ( "%s   %s %s   %s   %s   %s %s   %s",
      segment4[5] ? "#" : "-", segment4[1] ? "#" : "-",
      segment3[5] ? "#" : "-", segment3[1] ? "#" : "-",
      segment2[5] ? "#" : "-", segment2[1] ? "#" : "-",
      segment1[5] ? "#" : "-", segment1[1] ? "#" : "-");
    // g
    $display ( "%s %s   %s %s",
      segment4[6] ? " === " : " --- ",
      segment3[6] ? " === " : " --- ",
      segment2[6] ? " === " : " --- ",
      segment1[6] ? " === " : " --- ");
    // e, c
    $display ( "%s   %s %s   %s   %s   %s %s   %s",
      segment4[4] ? "#" : "-", segment4[2] ? "#" : "-",
      segment3[4] ? "#" : "-", segment3[2] ? "#" : "-",
      segment2[4] ? "#" : "-", segment2[2] ? "#" : "-",
      segment1[4] ? "#" : "-", segment1[2] ? "#" : "-");
    $display ( "%s   %s %s   %s   %s   %s %s   %s",
      segment4[4] ? "#" : "-", segment4[2] ? "#" : "-",
      segment3[4] ? "#" : "-", segment3[2] ? "#" : "-",
      segment2[4] ? "#" : "-", segment2[2] ? "#" : "-",
      segment1[4] ? "#" : "-", segment1[2] ? "#" : "-");
    $display ( "%s   %s %s   %s   %s   %s %s   %s",
      segment4[4] ? "#" : "-", segment4[2] ? "#" : "-",
      segment3[4] ? "#" : "-", segment3[2] ? "#" : "-",
      segment2[4] ? "#" : "-", segment2[2] ? "#" : "-",
      segment1[4] ? "#" : "-", segment1[2] ? "#" : "-");
    // d
    $display ( "%s %s   %s %s",
      segment4[3] ? " === " : " --- ",
      segment3[3] ? " === " : " --- ",
      segment2[3] ? " === " : " --- ",
      segment1[3] ? " === " : " --- ");
    $display;

  end
endtask


// 
//
// segment2ascii
// return the ascii character represented by the
// 7 segments.
//
//
function [7:0] segment2ascii (input [6:0]  segment);
  reg [15:0] 		  oneHot;
  reg [7:0] 		  bcd;
  begin
    oneHot[0] =  segment[0] &  segment[1] &  segment[2] &  segment[3] &  segment[4] &  segment[5] & ~segment[6];
    oneHot[1] = ~segment[0] &  segment[1] &  segment[2] & ~segment[3] & ~segment[4] & ~segment[5] & ~segment[6];
    oneHot[2] = segment[0]  &  segment[1] & ~segment[2] &  segment[3] &  segment[4] & ~segment[5] &  segment[6];
    oneHot[3] = segment[0]  &  segment[1] &  segment[2] &  segment[3] & ~segment[4] & ~segment[5] &  segment[6];
    oneHot[4] = ~segment[0] &  segment[1] &  segment[2] & ~segment[3] & ~segment[4] &  segment[5] &  segment[6];
    oneHot[5] = segment[0]  & ~segment[1] &  segment[2] &  segment[3] & ~segment[4] &  segment[5] &  segment[6];
    oneHot[6] = segment[0]  & ~segment[1] &  segment[2] &  segment[3] &  segment[4] &  segment[5] &  segment[6];
    oneHot[7] = segment[0]  &  segment[1] &  segment[2] & ~segment[3] & ~segment[4] & ~segment[5] & ~segment[6];
    oneHot[8] = segment[0]  &  segment[1] &  segment[2] &  segment[3] &  segment[4] &  segment[5] &  segment[6];
    oneHot[9] = segment[0]  &  segment[1] &  segment[2] & ~segment[3] & ~segment[4] &  segment[5] &  segment[6];
    oneHot[10]= segment[0]  &  segment[1] &  segment[2] & ~segment[3] &  segment[4] &  segment[5] &  segment[6];
    oneHot[11]= ~segment[0] & ~segment[1] &  segment[2] &  segment[3] &  segment[4] &  segment[5] &  segment[6];
    oneHot[12]= segment[0]  & ~segment[1] & ~segment[2] &  segment[3] &  segment[4] &  segment[5] & ~segment[6];
    oneHot[13]= ~segment[0] &  segment[1] &  segment[2] &  segment[3] &  segment[4] & ~segment[5] &  segment[6];
    oneHot[14]= segment[0]  & ~segment[1] & ~segment[2] &  segment[3] &  segment[4] &  segment[5] &  segment[6];
    oneHot[15]= segment[0]  & ~segment[1] & ~segment[2] & ~segment[3] &  segment[4] &  segment[5] &  segment[6];

    case (oneHot[15:0])
      16'b1000_0000_0000_0000 : bcd = "f";
      16'b0100_0000_0000_0000 : bcd = "e";
      16'b0010_0000_0000_0000 : bcd = "d";
      16'b0001_0000_0000_0000 : bcd = "c";
      16'b0000_1000_0000_0000 : bcd = "b";
      16'b0000_0100_0000_0000 : bcd = "a";
      16'b0000_0010_0000_0000 : bcd = "9";
      16'b0000_0001_0000_0000 : bcd = "8";
      16'b0000_0000_1000_0000 : bcd = "7";
      16'b0000_0000_0100_0000 : bcd = "6";
      16'b0000_0000_0010_0000 : bcd = "5";
      16'b0000_0000_0001_0000 : bcd = "4";
      16'b0000_0000_0000_1000 : bcd = "3";
      16'b0000_0000_0000_0100 : bcd = "2";
      16'b0000_0000_0000_0010 : bcd = "1";
      16'b0000_0000_0000_0001 : bcd = "0";
      default:	 bcd = ".";
    endcase
    segment2ascii = bcd;
  end
endfunction // segment2ascii

//
// convert ascii to binary
//
function [4:0] ascii2bin (input [7:0] t);
  reg [7:0] 	   bin8;    // 8 bit binary
  reg [4:0] 	   result;

  begin

    if ((t >= 8'h30) && (t <= 8'h3f)) begin
      bin8 = t - 8'h30;
      ascii2bin = {1'b0, bin8[3:0]};
    end
    else
      if ((t >= 8'h50) && (t <= 8'h5f)) begin
        bin8 = t - 8'h50;
        ascii2bin = {1'b1, bin8[3:0]};
      end
      else if ((t>= 8'h61) && (t <= 8'h66)) begin
        // a-f
        bin8 = t - 8'h61;
        ascii2bin = {1'b0, bin8[3:0] + 4'ha};
      end
      else if ((t>= 8'h41) && (t <= 8'h46)) begin
        // A-F
        bin8 = t - 8'h41;
        ascii2bin = {1'b1, bin8[3:0] + 4'ha};
      end
      else
      begin
        ascii2bin = 5'b0000;
      end
    end
  endfunction // ascii2bin



  //
  // print the "LEDS" to the screen
  //
  task displayLattice(input [4:0] leds);
    begin
      #1;
      $display("    [%c]", leds[2] ? "*":".");
      $display(" [%c][%c][%c] ", leds[1] ? "*":".", leds[4] ? "*" : ".", leds[3] ? "*" : ".");
      $display("    [%c]", leds[0] ? "*":".");
      //	 $display($time,,, ": %d  %d  %c  -> %d %d", a, b, op ? "-" : "+", leds[4], leds[3:0]);
      $display;
    end	 
  endtask


  //
  // sendByte
  // send a byte to the DUT
  //
  task sendByte(input [7:0] byt);
    begin
      @(posedge clk12m);
      tb_rx_data_rdy <= 1;
      tb_rx_data     <= byt;
      @(posedge clk12m);
      tb_rx_data_rdy <= 0;
    end
  endtask


  //
  // waitN
  // wait for N cycles
  //
  task waitN(input integer N);
    begin
      repeat (N) begin
        @(posedge clk12m);
      end
    end
  endtask // waitN



  //
  // print out a snipped of JSON for one test
  //
  task jsonTest(input integer firstOne, input integer tNum, input reg[`MAXMSG * 8-1:0] oStr, input integer score);
    begin
      $display("%c { \"name\" : \"test%d\",", (firstOne == 1'b1) ? " ": ",", tNum);
      $display("%-s", oStr);
      $display("\"score\" : %d}", score);
    end
  endtask

  reg go;


  initial begin

    // uncomment these two lines if using icarus verilog and gtkwave
    // $dumpfile("lab3.vcd");
    // $dumpvars(0, top_sft);
    go <= 0;
    tb_sim_rst <= 0;
    clk12m <= 0;
    tb_rx_data = 8'b0;
    tb_rx_data_rdy = 1'b0;
    #40
    tb_sim_rst <= 1;
    #40
    #40
    #40
    #40
    tb_sim_rst <= 0;
    go <= 1;
  end

  always @(*) begin
    #40;
    clk12m <= ~clk12m;
  end

  //   always @(leds) begin
  //      displayLattice(leds);
  //   end

  reg [6:0] segment1d;
  reg [6:0] segment2d;
  reg [6:0] segment3d;
  reg [6:0] segment4d;

  always @(segment1, segment2, segment3, segment4) begin
    segment1d <= segment1;
    segment2d <= segment2;
    segment3d <= segment3;
    segment4d <= segment4;
  end


  //  `define CHECKER
  `ifndef CHECKER
    always @(segment1d, segment2d, segment3d, segment4d) begin
      //displayLED(segment1d, segment2d, segment3d, segment4d);
      $display("%s %s %s %s", 
        segment2ascii(segment4d),
        segment2ascii(segment3d),
        segment2ascii(segment2d),
        segment2ascii(segment1d)
      );
    end
  `endif

  `ifdef CHECKER
    reg [3:0] Sones, exSones, nexSones;
    reg [3:0] Stens, exStens, nexStens;
    reg [3:0] Mones, exMones, nexMones;
    reg [3:0] Mtens, exMtens, nexMtens;
    reg misMatch;
    reg firstMismatch;

    initial begin
      exSones = 1;
      nexSones = 1;
      exStens = 0;
      nexStens = 0;
      exMones = 0;
      nexMones = 0;
      exMtens = 0;
      nexMtens = 0;
      firstMismatch = 0;
    end



    //
    // check for sequential increments
    // on the time (checkpoint test)
    //

    always @(segment1d, segment2d, segment3d, segment4d) begin
      if (go) begin
        Sones = ascii2bin(segment2ascii(segment1d));
        Stens = ascii2bin(segment2ascii(segment2d));
        Mones = ascii2bin(segment2ascii(segment3d));
        Mtens = ascii2bin(segment2ascii(segment4d));

        misMatch = 0;
        if (!(Sones == exSones))
          misMatch = 1;
        if (!(Stens == exStens))
          misMatch = 1;
        if (!(Mones == exMones))
          misMatch = 1;
        if (!(Mtens == exMtens))
          misMatch = 1;

        if (misMatch) begin
          $display("fail: expected %1d%1d:%1d%1d got %1d%1d:%1d%1d",
            exMtens, exMones, exStens, exSones,
            Mtens, Mones, Stens, Sones);
          exMtens = Mtens;
          exMones = Mones;
          exStens = Stens;
          exSones = Sones;
        end
        else
          $display("pass: expected %1d%1d:%1d%1d got %1d%1d:%1d%1d",
        exMtens, exMones, exStens, exSones,
        Mtens, Mones, Stens, Sones);
      misMatch = 0;

      nexSones = (exSones + 1) % 10;
      if (exSones == 9)
        nexStens = (exStens +1) % 6;
      else
        nexStens = exStens;

      if ((exSones == 9) && (exStens == 5))
        nexMones = (exMones + 1) % 10;
      else
        nexMones = exMones;

      if ((exSones == 9) && (exStens == 5)&& (exMones == 9))
        nexMtens = (exMtens + 1) % 6;
      else
        nexMtens = exMtens;

      exSones = nexSones;
      exStens = nexStens;
      exMones = nexMones;
      exMtens = nexMtens;
    end      
  end
`endif

//
// display digits coming back from the dut
//  display the segment displays
//
//      $display("%s %s  %s %s",
//	       segment2ascii(segment4d),
//	       segment2ascii(segment3d),
//	       segment2ascii(segment2d),
//	       segment2ascii(segment1d));

//  display the alarm display
//
//   always @(posedge ut_tx_data_rdy) begin
//	   #1;
//	   $display("%s", ut_tx_data);
//   end



// ------------------------
//
// stimulus
//
//
initial begin
  #400;
  #400;
  @(posedge clk12m);
  //      $display("{\"vtests\" : [");
  tb_rx_data = 8'b0;
  tb_rx_data_rdy = 1'b0;
  //
  waitN(100);
  `ifndef CHECKER
    sendByte("l");    // load time
    waitN(1);
    sendByte("5");
    sendByte("9");
    sendByte("5");
    sendByte("5");
    sendByte(8'h0d);
    waitN(5);
    sendByte("L");    // load alarm
    sendByte("0");
    sendByte("3");
    sendByte("2");
    sendByte("4");
    sendByte(8'h0d);
    waitN(100);
  `endif      
  //
  //      $display("]}");
  $finish;

end

integer testCount = 0;
integer errorCount = 0;
integer score = 1;
integer firstOne = 1;
//
// tests
//



endmodule // tb_sft
