// Mock Lattice iCE40 Oscillator
module SB_HFOSC (
    input       CLKHFEN,
    input       CLKHFPU,
    output reg  CLKHF
);
    parameter CLKHF_DIV = "0b00";
    initial CLKHF = 0;
    always #41.666 CLKHF = ~CLKHF; // ~12MHz
endmodule

// Mock Lattice iCE40 PLL (Pass-through)
module SB_PLL40_CORE (
    input   REFERENCECLK,
    output  PLLOUTCORE,
    output  PLLOUTGLOBAL,
    input   EXTFEEDBACK,
    input   DYNAMICDELAY,
    output  LOCK,
    input   BYPASS,
    input   RESETB,
    input   LATCHINPUTVALUE,
    input   SDI,
    input   SCLK,
    input   SHIFTREG_O
);
    parameter FEEDBACK_PATH = "SIMPLE";
    parameter PLLOUT_SELECT = "GENCLK";
    parameter DIVR = 4'b0000;
    parameter DIVF = 7'b0000000;
    parameter DIVQ = 3'b000;
    parameter FILTER_RANGE = 3'b000;

    assign PLLOUTCORE = REFERENCECLK;
    assign PLLOUTGLOBAL = REFERENCECLK;
    assign LOCK = 1'b1;
endmodule
