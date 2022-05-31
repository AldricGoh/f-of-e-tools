/*
 *	Description:
 *
 *		This module implements a latched clock gating circuit to "turn off" components
 *      which are not needed during the operation of a program in the processor
 *
 */

 module clk_gate(enable, clk, rst, clk_gated);

    input enable;
    input clk;
    input rst;
    output clk_gated;

    wire latch_out;

    // Negative Edge Clock with Synchronous Reset
    SB_DFFNSR SB_DFFNSR_inst (
      .Q(latch_out),    // Registered output
      .C(clk),    // Clock
      .D(enable),    // Data
      .R(rst)     // Synchronous reset
      );

    assign clk_gated = latch_out && clk ;

 endmodule