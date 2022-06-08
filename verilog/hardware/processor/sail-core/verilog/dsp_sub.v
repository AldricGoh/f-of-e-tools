module dsp_sub(input1, input2, out, carry_out);
  input [31:0] input1;
  input [31:0] input2;
  // input add_sub;
  output [31:0] out;
  output carry_out;

  // wire [31 : 0] alu_output;
	SB_MAC16 i_sbmac16
	( // port interfaces
		.A(input2[31 : 16]),
		.B(input2[15 : 0]),
		.C(input1[31 : 16]),
		.D(input1[15 : 0]),
		.O(out),
		.CLK(), // HERE
		.CE(1'b0),
		.IRSTTOP(1'b0),
		.IRSTBOT(1'b0),
		.ORSTTOP(1'b0),
		.ORSTBOT(1'b0),
		.AHOLD(1'b0),
		.BHOLD(1'b0),
		.CHOLD(1'b0),
		.DHOLD(1'b0),
		.OHOLDTOP(1'b0),
		.OHOLDBOT(1'b0),
		.OLOADTOP(1'b0),
		.OLOADBOT(1'b0),
		.ADDSUBTOP(1'b1),
		.ADDSUBBOT(1'b1),
		.CO(carry_out),
		.CI(1'b0),
		.ACCUMCI(1'b0),
		.ACCUMCO(),
		.SIGNEXTIN(1'b0),
		.SIGNEXTOUT()
	);
	defparam i_sbmac16.NEG_TRIGGER = 1'b0;
	defparam i_sbmac16.C_REG = 1'b0;
	defparam i_sbmac16.A_REG = 1'b0;
	defparam i_sbmac16.B_REG = 1'b0;
	defparam i_sbmac16.D_REG = 1'b0;

	defparam i_sbmac16.TOP_8x8_MULT_REG = 1'b0;
	defparam i_sbmac16.BOT_8x8_MULT_REG = 1'b0;
	defparam i_sbmac16.PIPELINE_16x16_MULT_REG1 = 1'b0;
	defparam i_sbmac16.PIPELINE_16x16_MULT_REG2 = 1'b0;

	defparam i_sbmac16.TOPOUTPUT_SELECT = 2'b00; // accum register output at O[31:16]
	defparam i_sbmac16.TOPADDSUB_LOWERINPUT = 2'b00;
	defparam i_sbmac16.TOPADDSUB_UPPERINPUT = 1'b1;
	defparam i_sbmac16.TOPADDSUB_CARRYSELECT = 2'b10;
	defparam i_sbmac16.BOTOUTPUT_SELECT = 2'b00; // accum regsiter output at O[15:0]
	defparam i_sbmac16.BOTADDSUB_LOWERINPUT = 2'b00;
	defparam i_sbmac16.BOTADDSUB_UPPERINPUT = 1'b1;
	defparam i_sbmac16.BOTADDSUB_CARRYSELECT = 2'b00;
	defparam i_sbmac16.MODE_8x8 = 1'b1;
	defparam i_sbmac16.A_SIGNED = 1'b0;
	defparam i_sbmac16.B_SIGNED = 1'b0;
endmodule