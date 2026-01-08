// --- Program to display multiples of a specified positive integer ---
#include <stdint.h>
#include "io.h"
#define NUM 3 // Specified integer

// Function to transmit message through UART
void print_uart(const char *str) {
    while (*str) {
        while (IO_IN(UART_CNTL));
        IO_OUT(UART_DATA, *str++);
    }
}

// Simple delay function
void delay(int cycles) {
    volatile int i;
    for (i = 0; i < cycles; i++);
}

// Counter increment function
void inc(int* counter) {
    (*counter)++;
    if (*counter > 15) *counter = 0;
}

void main() {
    delay(5000000);
    print_uart("\n--- Multi-Register GPIO IP Test ---\n");
    print_uart("\nGuess the integer whose multiples are being displayed in binary below:\n");

    int counter = 0;
    while (1) {
        // Configure pin directions (1 - Output, 0 - Input)
        if ((counter % NUM) == 0) {
            IO_OUT(GPIO_DIR, 0xF); // 1111
        } else {
            IO_OUT(GPIO_DIR, 0x0); // 0000
            inc(&counter);
            continue;
        }

        // Write data to register
        IO_OUT(GPIO_DATA, counter & 0xF);

        // Read back from register
        uint32_t read_val = IO_IN(GPIO_READ);

        // Send status through UART
        if (read_val & 0x8) print_uart("1"); else print_uart("0");
        if (read_val & 0x4) print_uart("1"); else print_uart("0");
        if (read_val & 0x2) print_uart("1"); else print_uart("0");
        if (read_val & 0x1) print_uart("1\n"); else print_uart("0\n");

        // Increment counter
        inc(&counter);

        delay(500000); // Delay for visual blink
    }
}
