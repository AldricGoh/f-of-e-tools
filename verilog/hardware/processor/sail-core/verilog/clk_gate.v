/*
 *	Description:
 *
 *		This module implements a latched clock gating circuit to "turn off" components
 *      which are not needed during the operation of a program in the processor
 *
 */

`ifdef SIMULATION

  module clk_gate(enable, clk, clk_gated);
    input enable;
    input clk;
    output clk_gated;

    assign clk_gated = enable & clk;
  endmodule

`else

  module clk_gate(enable, clk, clk_gated);
    input enable;
    input clk;
    output clk_gated;

    // SB_DFFNES - D Flip-Flop – Negative Edge Clock, Set is asynchronous on falling clock edge with clock Enable.
    SB_DFFNES   SB_DFFNES_inst (
        .Q(clk_gated),            // Registered Output
        .C(clk),            // Clock
        .E(enable),            // Clock Enable
        .D(clk),            // Data
        .S(1'b0)             // Asynchronously Set
      );

  endmodule

`endif
