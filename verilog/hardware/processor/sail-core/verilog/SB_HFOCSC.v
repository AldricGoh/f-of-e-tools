module SB_HFOSC #(parameter CLKHF_DIV = "0b11") (
    CLKHFEN, CLKHFPU, CLKHF
);
    input CLKHFEN;
    input CLKHFPU;
    output CLKHF;

    reg clk = 0;
    always #1 clk = !clk;

    assign CLKHF = clk;
endmodule
