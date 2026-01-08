#include "io.h"
#include "uart.h" // Custom UART Header File

// Delay function
void delay(int cycles) {
    for (volatile int i = 0; i < cycles; i++);
}

void main() {
    uprint("\n\t\t\t\t--- PWM IP Test ---\n");    

    // Configure PWM
    IO_OUT(PWM_PERIOD, 1000);   // Set Period to 1000 ticks
    IO_OUT(PWM_CTRL, 1);        // Enable = 1, Polarity = 0 (Active HIGH)
    uprint("\nConfiguring PWM: Period=%d, Mode='Active HIGH'\n",1000);

    uint32_t status = IO_IN(PWM_STATUS);
    uprint("PWM Enabled. Current Status: 0x%x\n",status);

    // Breathe LED effect
    int duty = 0;
    int direction = 1;
    int count = 0;
    
    while (count < 6) { 
        // Breathe 6 times 
        /* 3 times with Polarity = 0
         * 3 times with Polarity = 1 */

        if (count == 3 && duty == 0) {
            IO_OUT(PWM_CTRL, 3); // Change Polarity
            uprint("\nPolarity Inverted. Current Status: 0x%x. Mode='Active LOW'\n",status);
        }

        // Update Duty Cycle
        IO_OUT(PWM_DUTY, duty);
        delay(1000); // Wait a bit so changes are visible

        if (direction) {
            duty += 100;
            if (duty >= 1000) direction = 0;
        } else {
            duty -= 100;
            if (duty <= 0) { direction = 1; count++; }
        }
    }
    
    // Disable PWM
    IO_OUT(PWM_CTRL, 0);
    uprint("\nPWM Disabled.\n");
    delay(5000000); // Wait for few seconds to observe disability
    while(1);   // Prevent 'main' from returning  
}
