
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
  // uncomment this define if you want to target hardware
  // otherwise, this file will be configured for the simulator
  //
  `define HW

  //
  // Revision History : 0.0
  //

  //
  //  decodeKeys
  //
  // decode the input from the uart (receiver) and decode valid
  // characters.
  // 
  //






  //
  // gl_fsm
  //
  // 4 state state machine.
  // we capture 3 characters if they are legal.
  // illegal characters are ignored and the state machine stays
  // put.
  //
  module gl_fsm(
    output reg enR11, // load first byte of r1
    output reg enR12, // load second byte of r1
    output reg enR21, // load first byte of r2
    output reg enR22, // load second byte of r2
    output reg enOp, // load op
    output reg Gl_adder_start,
    input validInput,
    input validOp,
    input bu_tx_busy,
    input clk,
    input Gl_rst
    `ifdef HW
    `else
      ,tb_sim_rst
    `endif
    );

    reg [2:0] 	       state;
    reg [2:0] 	       nxtState;

    localparam IDLE = 3'b000, GOTR11 = 3'b001, GOTR12 = 3'b010, 
      GOTR21 = 3'b100, GOTR22 = 3'b101, DOOP = 3'b110;


    always @(*) begin
      enR11 = 1'b0;
      enR12 = 1'b0;
      enR21 = 1'b0;
      enR22 = 1'b0;
      enOp = 1'b0;
      Gl_adder_start = 1'b0;
      case (state)
        IDLE: 
        begin
          if (validInput) begin
            nxtState = GOTR11;
            enR11 = 1'b1;
          end
          else
            nxtState = IDLE;
        end
        GOTR11:
        begin
          if (validInput) begin
            nxtState = GOTR12;
            enR12 = 1'b1;
          end
          else
            nxtState = GOTR11;
        end
        GOTR12:
        begin
          if (validInput) begin
            nxtState = GOTR21;
            enR21 = 1'b1;
          end
          else
            nxtState = GOTR12;
        end
        GOTR21:
        begin
          if (validInput) begin
            nxtState = GOTR22;
            enR22 = 1'b1;
          end
          else
            nxtState = GOTR21;
        end
        GOTR22:
        begin
          if (validOp) begin
            nxtState = DOOP;
            enOp = 1'b1;
          end
          else
            nxtState = GOTR22;
        end
        DOOP:
        begin
          nxtState = IDLE;
          Gl_adder_start = 1;
        end
      endcase // case (state)
    end // always @ (*)

    always @(posedge clk) begin
      if (Gl_rst) begin
        state <= IDLE;
      end
      else begin
        state <= nxtState;
      end
    end      
    endmodule


    `ifdef HW
      module inpin(
        input clk,
        input pin,
        output rd);

      SB_IO #(.PIN_TYPE(6'b0000_00)) _io (
        .PACKAGE_PIN(pin),
        .INPUT_CLK(clk),
        .D_IN_0(rd));
      endmodule
    `endif


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
      input 		   L2_adder_rdy1,
      input [7:0] 	   L2_adder_data2,
      input 		   L2_adder_rdy2,
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


        reg p1;
        reg p2;

        always @(posedge clk) begin
          if ( L2_adder_rdy2 == 1'b1)
            p1 <= 1'b1;
          else
            p1 <= 1'b0;
        end
     
        always @(posedge clk) begin
          if ( p1 == 1'b1)
            p2 <= 1'b1;
          else
            p2 <= 1'b0;
        end


        //
        // transmit uart
        // echo input characters and output from adder
        //
        
        always @(posedge clk) begin
          Gl_tx_data <= is_hexplus ? bu_rx_data : p1 ? L2_adder_data1 : L2_adder_data2;
          Gl_tx_data_rdy <= is_hexplus | L2_adder_rdy1 | L2_adder_rdy2;
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


        assign led[0] = L2_led[0];
        assign led[1] = L2_led[1];
        assign led[2] = L2_led[2];
        assign led[3] = L2_led[3];
        assign led[4] = L2_led[4];



        endmodule

        //
        // top level for lab2
        //
        module latticehx1k(
          output 	sd,

          input 	clk_in,
          input wire 	from_pc,
          output wire 	to_ir,
          output wire 	o_serial_data,
          output 	test1,
          output 	test2,
          output 	test3,
          output [4:0] led

          // for software only

          `ifdef HW
          `else
            ,input 	tb_sim_rst
            ,input [7:0] 	tb_rx_data // pretend data coming from the uart
            ,input 	tb_rx_data_rdy //

            ,output [7:0] ut_tx_data // shortcut, data from the fake tx uart
            ,output 	ut_tx_data_rdy
          `endif

          );



          wire 			clk;
          `ifdef HW
            wire tb_sim_rst = 1'b0;

            assign to_ir = 1'b0;
            assign sd = 1'b0;
            assign test1 = 1'b0;
            assign test2 = 1'b0;
            assign test3 = 1'b0;
            // for simple feedback path
            // pllout =   Frefclock * (DIVF+1) / [ 2^divq x (divr+1)]
            //  12 MHz * 4   / [ 2^0 x 1] = 48 MHz    // doesn't work in icecube2
            //  12 MHz * 8   / [ 2^1 x 1] = 48 MHz    // doesn't work in icecube2
            //  12 MHz * 64   / [ 2^4 x 1] = 48 MHz    // doesn't work in icecube2
            //  

            defparam ice_pll_inst.DIVR = 4'b0000;
            defparam ice_pll_inst.DIVF = 7'b0111111;


            defparam ice_pll_inst.DIVQ = 3'b100;
            defparam ice_pll_inst.FILTER_RANGE = 3'b001;
            defparam ice_pll_inst.FEEDBACK_PATH = "SIMPLE";
            defparam ice_pll_inst.DELAY_ADJUSTMENT_MODE_FEEDBACK = "FIXED";
            defparam ice_pll_inst.FDA_FEEDBACK = 4'b0000;
            defparam ice_pll_inst.DELAY_ADJUSTMENT_MODE_RELATIVE = "FIXED";
            defparam ice_pll_inst.FDA_RELATIVE = 4'b0000;
            defparam ice_pll_inst.SHIFTREG_DIV_MODE = 2'b00;
            defparam ice_pll_inst.PLLOUT_SELECT = "GENCLK";
            defparam ice_pll_inst.ENABLE_ICEGATE = 1'b0;




            SB_PLL40_CORE ice_pll_inst(
              .REFERENCECLK(clk_in),
              .PLLOUTCORE(clk),
              //.PLLOUTGLOBAL(clk),
              // .LOCK(D5),
              .EXTFEEDBACK(),
              .DYNAMICDELAY(),
              .RESETB(1'b1),
              .BYPASS(1'b0),
              .LATCHINPUTVALUE(),
              .LOCK(),
              .SDI(),
              .SDO(),
              .SCLK()
            );

          `else // !`ifdef HW
            fake_pll uut (
              .REFERENCECLK(clk_in),
              .PLLOUTCORE(clk),
              .PLLOUTGLOBAL(),
              .RESETB(1'b1),
              .BYPASS(1'b0)
            );

          `endif

          wire 			Gl_rst;           // soft internal reset
          wire 			Gl_adder_start;  // start add/sub (one cycle pulse)
          wire 			Gl_subtract;     // is operation subtract
          wire [7:0] 			Gl_r11;		 // operand 1_1
          wire [7:0] 			Gl_r12;		 // operand 1_2
          wire [7:0] 			Gl_r21;		 // operand 2_1
          wire [7:0] 			Gl_r22;		 // operand 2_2
          wire [7:0] 			L2_led;       // output from Lab2_140L

          wire [7:0] 			L2_adder_data1;    // adder data1
          wire 			L2_adder_rdy1;     // adder data1 is ready
          wire [7:0] 			L2_adder_data2;    // adder data2
          wire 			L2_adder_rdy2;     // adder data2 is ready
          wire [7:0] 			Gl_tx_data;
          wire 			Gl_tx_data_rdy;


          wire [7:0] 	    bu_rx_data;         // data from uart to dev
          wire  	    bu_rx_data_rdy;     // data from uart to dev is valid
          wire             bu_tx_busy;         // uart is busy transmitting

          `ifdef HW

            wire 	    uart_RXD;

            inpin _rcxd(.clk(clk), .pin(from_pc), .rd(uart_RXD));

            // print buffer
            uartTxBuf ufifo (
              .utb_txdata(utb_txdata),
              .utb_txdata_rdy(utb_tx_data_rdy),
              .txdata(Gl_tx_data),
              .txDataValid(Gl_tx_data_rdy),
              .txBusy(bu_tx_busy),
              .rxdata(bu_rx_data),
              .rxDataValid(bu_rx_data_rdy),
              .printBuf(0),
              .reset(rst),
              .clk(clk)
            );

            // with 48 MHz clock, 460800 baud, 8, N, 1
            buart buart (
              .clk (clk),
              .resetq(1'b1),
              .rx(uart_RXD),
              .tx(o_serial_data),
              .rd(1'b1),                // read strobe
              .wr(utb_tx_data_rdy),	  // write strobe 
              .valid(bu_rx_data_rdy),   // rx has valid data
              .tx_data(utb_tx_data),
              .rx_data(bu_rx_data));

          `else // !`ifdef HW
            fake_buart buart (
              .clk (clk),
              .resetq(tb_sim_rst),
              .rx(from_pc),
              .tx(o_serial_data),
              .rd(1'b1),                // read strobe
              .wr(Gl_tx_data_rdy),                    // write strobe 
              .valid(bu_rx_data_rdy),   // rx has valid data
              .busy(bu_tx_busy),        // uart is busy transmitting
              .tx_data(Gl_tx_data),
              .rx_data(bu_rx_data),
              .fa_data(ut_tx_data),
              .fa_valid(ut_tx_data_rdy),
              .to_dev_data(tb_rx_data),
              .to_dev_data_valid(tb_rx_data_rdy));
          `endif

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
            .L2_adder_rdy1(L2_adder_rdy1),
            .L2_adder_rdy2(L2_adder_rdy2),
            .Gl_tx_data(Gl_tx_data),
            .Gl_tx_data_rdy(Gl_tx_data_rdy),

            .Gl_rst(Gl_rst),
            .L2_led(L2_led),
            .led(led),
            .tb_sim_rst(tb_sim_rst),
            .clk(clk));



          Lab2_140L Lab_UT(
            .Gl_rst   (Gl_rst),                              // reset signal
            .clk      (clk),
            .Gl_adder_start (Gl_adder_start),
            .Gl_subtract (Gl_subtract), 
            .Gl_r11    (Gl_r11[7:0]),
            .Gl_r12    (Gl_r12[7:0]),
            .Gl_r21    (Gl_r21[7:0]),
            .Gl_r22    (Gl_r22[7:0]),
            .L2_adder_data1   (L2_adder_data1),
            .L2_adder_data2   (L2_adder_data2),
            .L2_adder_rdy1 (L2_adder_rdy1),
            .L2_adder_rdy2 (L2_adder_rdy2),
            .L2_led   (L2_led[7:0])        //output LED
          );



          endmodule // latticehx1k
