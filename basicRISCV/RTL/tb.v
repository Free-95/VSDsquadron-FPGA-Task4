`timescale 1ns/1ns
module testbench;
    reg RESET;
    reg RXD;
    wire [3:0] LEDS;
    wire TXD;
    wire PWM;    

    SOC uut (
        .RESET(RESET),
        .LEDS(LEDS), 
        .RXD(RXD),
        .TXD(TXD),
        .PWM(PWM)
    );

    initial begin
        $dumpfile("pwm_test.vcd");
        $dumpvars(0, testbench);

        // Init
        RXD = 1;
        RESET = 0;

        // Reset Pulse
        #100 RESET = 1;
        #100 RESET = 0; 

        #600000000;
        $finish;
    end
endmodule
