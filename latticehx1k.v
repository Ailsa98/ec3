
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
  module decodeKeys(
    output      de_esc,        // esc char detected
    output      de_hexplus,    // hex char + ;:<=>? detected
    output      de_pn,         // detected + or -
    input [7:0] charData,      // ascii coming in
    input       charDataValid  // valid ascii
  );


  wire 		      is_b0_1 = charData[0];
  wire 		      is_b1_1 = charData[1];
  wire 		      is_b2_1 = charData[2];
  wire 		      is_b3_1 = charData[3];
  wire 		      is_b4_1 = charData[4];
  wire 		      is_b5_1 = charData[5];
  wire 		      is_b6_1 = charData[6];
  wire 		      is_b7_1 = charData[7];
  wire 		      is_b0_0 = ~charData[0];
  wire 		      is_b1_0 = ~charData[1];
  wire 		      is_b2_0 = ~charData[2];
  wire 		      is_b3_0 = ~charData[3];
  wire 		      is_b4_0 = ~charData[4];
  wire 		      is_b5_0 = ~charData[5];
  wire 		      is_b6_0 = ~charData[6];
  wire 		      is_b7_0 = ~charData[7];


  // esc - 1b
  assign de_esc = &{is_b7_0, is_b6_0, is_b5_0, is_b4_1,
    is_b3_1, is_b2_0, is_b1_1, is_b0_1} & charDataValid;

  // 0-9
  // 0-9:;<=>?
  wire de_num = &{is_b7_0, is_b6_0 , is_b5_1, is_b4_1} & charDataValid;      



  // a-f
  assign de_hexplus = (&{is_b7_0, is_b6_1, is_b5_1, is_b4_0, is_b3_0} & (~&{is_b2_1, is_b1_1, is_b0_1}) 
  | de_num) & charDataValid;

  // +, -  0x2b or 0x2d
  assign de_pn = (&{is_b7_0, is_b6_0, is_b5_1, is_b4_0, is_b3_1, is_b0_1} & (charData[2] ^ charData[1])) &
    charDataValid;

  endmodule


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

      wire        utb_tx_data;
      wire        utb_tx_data_rdy;

      `ifdef HW

        wire 	    uart_RXD;

        inpin _rcxd(.clk(clk), .pin(from_pc), .rd(uart_RXD));

        // print buffer
        uartTxBuf ufifo (
          .utb_txdata(utb_txdata),
          .utb_txdata_rdy(utb_tx_data_rdy),
          .txdata(gl_tx_data),
          .txDataValid(gl_tx_data_rdy),
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
          .wr(utb_txdata_rdy),	  // write strobe 
          .valid(bu_rx_data_rdy),   // rx has valid data
          .busy(bu_tx_busy),
          .tx_data(utb_tx_data),
          .rx_data(bu_rx_data));

      `else // !`ifdef HW
        // print buffer
        uartTxBuf ufifo (
          .utb_txdata(utb_tx_data),
          .utb_txdata_rdy(utb_tx_data_rdy),
          .txdata(gl_tx_data),
          .txDataValid(gl_tx_data_rdy),
          .txBusy(bu_tx_busy),
          .rxdata(bu_rx_data),
          .rxDataValid(bu_rx_data_rdy),
          .printBuf(0),
          .reset(rst),
          .clk(clk)
        );

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
          .fa_data(utb_tx_data),
          .fa_valid(ut_tx_data_rdy),
          .to_dev_data(tb_rx_data),
          .to_dev_data_valid(tb_rx_data_rdy));
      `endif

   wire [4:0] 	    L2_led;
   assign led[4:0] = L2_led[4:0];


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
