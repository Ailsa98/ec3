module top_sft;

   wire       tb_sim_rst;        // simulation only reset
   wire       clk12m;	 	 // 12 mhz clock
   wire [4:0] leds;              // lattice leds
   wire [7:0] tb_rx_data;
   wire       tb_rx_data_rdy;
   wire [7:0] L3_tx_data;
   wire       L3_tx_data_rdy;
   wire [6:0] L3_segment1;
   wire [6:0] L3_segment2;
   wire [6:0] L3_segment3;
   wire [6:0] L3_segment4;
   wire [3:0] di_Mtens;
   wire [3:0] di_Mones;
   wire [3:0] di_Stens;
   wire [3:0] di_Sones;
   wire [3:0] di_AMtens;
   wire [3:0] di_AMones;
   wire [3:0] di_AStens;
   wire [3:0] di_ASones;
   
   
   latticehx1k latticehx1k(
			   .sd(),
			   .clk_in(clk12m),
			   .from_pc(),
			   .to_ir(),
			   .o_serial_data(),
			   .led(leds),

			   .tb_sim_rst(tb_sim_rst),
			   .tb_rx_data(tb_rx_data),
			   .tb_rx_data_rdy(tb_rx_data_rdy),
			   .L3_tx_data(L3_tx_data),
			   .L3_tx_data_rdy(L3_tx_data_rdy),
			   .L3_segment1(L3_segment1),
			   .L3_segment2(L3_segment2),
			   .L3_segment3(L3_segment3),
			   .L3_segment4(L3_segment4),
			   .di_Mtens(di_Mtens),
			   .di_Mones(di_Mones),
			   .di_Stens(di_Stens),
			   .di_Sones(di_Sones),
			   .di_AMtens(di_AMtens),
			   .di_AMones(di_AMones),
			   .di_AStens(di_AStens),
			   .di_ASones(di_ASones)
			   
			   );
   

   tb_sft tb_sft (
	          .tb_sim_rst(tb_sim_rst),
		  .clk12m(clk12m),
		  .L3_tx_data(L3_tx_data),
		  .L3_tx_data_rdy(L3_tx_data_rdy),
		  .tb_rx_data(tb_rx_data),
		  .tb_rx_data_rdy(tb_rx_data_rdy),
		  .leds(leds),
		  .segment1(L3_segment1),
		  .segment2(L3_segment2),
		  .segment3(L3_segment3),
		  .segment4(L3_segment4)
		  );

 	     

endmodule // top_sft
