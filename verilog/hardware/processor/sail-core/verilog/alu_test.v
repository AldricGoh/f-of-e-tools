/*
	Authored 2018-2019, Ryan Voo.

	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"



/*
 *	Description:
 *
 *		This module implements the ALU for the RV32I.
 */



/*
 *	Not all instructions are fed to the ALU. As a result, the ALUctl
 *	field is only unique across the instructions that are actually
 *	fed to the ALU.
 */
module alu(ALUctl, A, B, ALUOut, Branch_Enable);
	input [6:0]		ALUctl;
	input [31:0]		A;
	input [31:0]		B;
	output reg [31:0]	ALUOut;
	output reg		Branch_Enable;

	reg [31:0]	C;
	reg [31:0]	D;

	wire [31 : 0] alu_output_sub;
	wire [31 : 0] alu_output_add;
	wire carry_out;


	// Initialise and configure DSPs 

	SB_MAC16 i_sbmac16_add
	(
		.A(A[31 : 16]),
		.B(A[15 : 0]),
		.C(B[31 : 16]),
		.D(B[15 : 0]),
		.O(alu_output_add),
		.CLK(),
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
		.ADDSUBTOP(1'b0),
		.ADDSUBBOT(1'b0),
		.CO(),
		.CI(1'b0),
		.ACCUMCI(1'b0),
		.ACCUMCO(),
		.SIGNEXTIN(1'b0),
		.SIGNEXTOUT()
	);

	SB_MAC16 i_sbmac16_sub
	(
		.A(B[31 : 16]),
		.B(B[15 : 0]),
		.C(A[31 : 16]),
		.D(A[15 : 0]),
		.O(alu_output_sub),
		.CLK(),
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

	defparam i_sbmac16_add.NEG_TRIGGER = 1'b0;
	defparam i_sbmac16_add.C_REG = 1'b0;
	defparam i_sbmac16_add.A_REG = 1'b0;
	defparam i_sbmac16_add.B_REG = 1'b0;
	defparam i_sbmac16_add.D_REG = 1'b0;

	defparam i_sbmac16_add.TOP_8x8_MULT_REG = 1'b0;
	defparam i_sbmac16_add.BOT_8x8_MULT_REG = 1'b0;
	defparam i_sbmac16_add.PIPELINE_16x16_MULT_REG1 = 1'b0;
	defparam i_sbmac16_add.PIPELINE_16x16_MULT_REG2 = 1'b0;

	defparam i_sbmac16_add.TOPOUTPUT_SELECT = 2'b00;
	defparam i_sbmac16_add.TOPADDSUB_LOWERINPUT = 2'b00;
	defparam i_sbmac16_add.TOPADDSUB_UPPERINPUT = 1'b1;
	defparam i_sbmac16_add.TOPADDSUB_CARRYSELECT = 2'b10;
	defparam i_sbmac16_add.BOTOUTPUT_SELECT = 2'b00;
	defparam i_sbmac16_add.BOTADDSUB_LOWERINPUT = 2'b00;
	defparam i_sbmac16_add.BOTADDSUB_UPPERINPUT = 1'b1;
	defparam i_sbmac16_add.BOTADDSUB_CARRYSELECT = 2'b00;
	defparam i_sbmac16_add.MODE_8x8 = 1'b1;
	defparam i_sbmac16_add.A_SIGNED = 1'b0;
	defparam i_sbmac16_add.B_SIGNED = 1'b0;

	defparam i_sbmac16_sub.NEG_TRIGGER = 1'b0;
	defparam i_sbmac16_sub.C_REG = 1'b0;
	defparam i_sbmac16_sub.A_REG = 1'b0;
	defparam i_sbmac16_sub.B_REG = 1'b0;
	defparam i_sbmac16_sub.D_REG = 1'b0;

	defparam i_sbmac16_sub.TOP_8x8_MULT_REG = 1'b0;
	defparam i_sbmac16_sub.BOT_8x8_MULT_REG = 1'b0;
	defparam i_sbmac16_sub.PIPELINE_16x16_MULT_REG1 = 1'b0;
	defparam i_sbmac16_sub.PIPELINE_16x16_MULT_REG2 = 1'b0;

	defparam i_sbmac16_sub.TOPOUTPUT_SELECT = 2'b00;
	defparam i_sbmac16_sub.TOPADDSUB_LOWERINPUT = 2'b00;
	defparam i_sbmac16_sub.TOPADDSUB_UPPERINPUT = 1'b1;
	defparam i_sbmac16_sub.TOPADDSUB_CARRYSELECT = 2'b10;
	defparam i_sbmac16_sub.BOTOUTPUT_SELECT = 2'b00;
	defparam i_sbmac16_sub.BOTADDSUB_LOWERINPUT = 2'b00;
	defparam i_sbmac16_sub.BOTADDSUB_UPPERINPUT = 1'b1;
	defparam i_sbmac16_sub.BOTADDSUB_CARRYSELECT = 2'b00;
	defparam i_sbmac16_sub.MODE_8x8 = 1'b1;
	defparam i_sbmac16_sub.A_SIGNED = 1'b0;
	defparam i_sbmac16_sub.B_SIGNED = 1'b0;

	

	/*
	 *	This uses Yosys's support for nonzero initial values:
	 *
	 *		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
	 *
	 *	Rather than using this simulation construct (`initial`),
	 *	the design should instead use a reset signal going to
	 *	modules in the design.
	 */
	initial begin
		ALUOut = 32'b0;
		Branch_Enable = 1'b0;
	end

	always @(ALUctl, A, B) begin
		case (ALUctl[3:0])
			/*
			 *	AND (the fields also match ANDI and LUI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_AND:	ALUOut = A & B;

			/*
			 *	OR (the fields also match ORI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_OR:	ALUOut = A | B;

			/*
			 *	ADD (the fields also match AUIPC, all loads, all stores, and ADDI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_ADD:	ALUOut = alu_output_add;

			/*
			 *	SUBTRACT (the fields also matches all branches)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB:	ALUOut = alu_output_sub;

			/*
			 *	SLT (the fields also matches all the other SLT variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLT:	ALUOut = $signed(A) < $signed(B) ? 32'b1 : 32'b0;

			/*
			 *	SRL (the fields also matches the other SRL variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRL:	ALUOut = A >> B[4:0];

			/*
			 *	SRA (the fields also matches the other SRA variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRA:	ALUOut = $signed(A) >>> B[4:0];

			/*
			 *	SLL (the fields also match the other SLL variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLL:	ALUOut = A << B[4:0];

			/*
			 *	XOR (the fields also match other XOR variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_XOR:	ALUOut = A ^ B;

			/*
			 *	CSRRW  only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRW:	ALUOut = A;

			/*
			 *	CSRRS only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRS:	ALUOut = A | B;

			/*
			 *	CSRRC only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRC:	ALUOut = (~A) & B;

			/*
			 *	Should never happen.
			 */
			default:					ALUOut = 0;
		endcase
	end

	always @(ALUctl, ALUOut, A, B) begin
		case (ALUctl[6:4])
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ:	Branch_Enable = (ALUOut == 0);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE:	Branch_Enable = !(ALUOut == 0);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT:	Branch_Enable = ($signed(A) < $signed(B));
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE:	Branch_Enable = ($signed(A) >= $signed(B));
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU:	Branch_Enable = (~carry_out);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU:	Branch_Enable = (carry_out);

			default:					Branch_Enable = 1'b0;
		endcase
	end
endmodule
