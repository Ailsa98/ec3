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

