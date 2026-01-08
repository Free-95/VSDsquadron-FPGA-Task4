#include <stdint.h>

// IO Base Address
#define IO_BASE      0x400000

// GPIO Offsets
#define GPIO_DATA    0x00
#define GPIO_DIR     0x04
#define GPIO_READ    0x08

// UART Offsets 
#define UART_DATA    0x10
#define UART_CNTL    0x14

// PWM  Offsets
#define PWM_CTRL     0x20
#define PWM_PERIOD   0x24
#define PWM_DUTY     0x28
#define PWM_STATUS   0x2C

// Access Macros
#define IO_IN(offset)     (*(volatile uint32_t*)(IO_BASE + (offset)))
#define IO_OUT(offset,val) (*(volatile uint32_t*)(IO_BASE + (offset)) = (val))
