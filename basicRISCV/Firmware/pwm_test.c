#include "io.h"
#include "uart.h" // Custom UART Header File

// Delay function
void delay(int cycles) {
    for (volatile int i = 0; i < cycles; i++);
}

void main() {
    uprint("\n\t\t\t\t--- PWM IP Test ---\n");    

    // Configure PWM
    int polarity = 0;                          // Polarity = 0 (Active HIGH)
    const char *mode = (polarity == 1) ? "Active LOW" : "Active HIGH";
    IO_OUT(PWM_PERIOD, 1000);                  // Set Period to 1000 ticks
    IO_OUT(PWM_CTRL, polarity * 2 + 1);        // Enable = 1 
    uprint("\nConfiguring PWM: Period=%d, Mode=%s\n", 1000, mode);

    uint32_t status = IO_IN(PWM_STATUS);
    uprint("PWM Enabled. Current Status: 0x%x\n", status);

    // Breathe LED effect
    int duty = 0;
    int direction = 1;
    int count = 0;

    jump:
    while (count < 3) { // Breathe 3 times 
        // Update Duty Cycle
        IO_OUT(PWM_DUTY, duty);
        delay(10000); // Wait a bit so changes are visible

        if (direction) {
            duty += 10;
            if (duty >= 1000) direction = 0;
        } else {
            duty -= 10;
            if (duty <= 0) { direction = 1; count++; }
        }
    }
    
    // Disable PWM
    IO_OUT(PWM_CTRL, 0);
    status = IO_IN(PWM_STATUS);
    uprint("\nPWM Disabled. Current Status: 0x%x\n", status);
    delay(500000); // Wait for few seconds to observe disability

    // Invert the polarity
    count = 0;
    polarity = (polarity + 1) % 2;
    mode = (polarity == 1) ? "Active LOW" : "Active HIGH";
    IO_OUT(PWM_CTRL, polarity * 2 + 1);
    status = IO_IN(PWM_STATUS);
    uprint("\nPolarity Inverted. Current Status: 0x%x. Mode=%s\n", status, mode);

    goto jump;    // Repeat the same process  
}
