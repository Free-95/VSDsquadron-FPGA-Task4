#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdarg.h>
#include "io.h" 

/* Low-Level UART Primitive 
 * Uses UART_CNTL and UART_DATA from io.h
 */
static void putchar(char c) {
    // Wait until UART is ready
    while (IO_IN(UART_CNTL)); 
    IO_OUT(UART_DATA, c);
}

/* Helper: Print a null-terminated string 
 */
static void print_string(const char* s) {
   for(const char* p = s; *p; ++p) {
      putchar(*p);
   }
}

/* Helper: Print integer in Decimal 
 */
static void print_dec(int val) {
   char buffer[255];
   char *p = buffer;
   if(val < 0) {
      putchar('-');
      print_dec(-val);
      return;
   }
   if (val == 0) {
       putchar('0');
       return;
   }
   while (val || p == buffer) {
      *(p++) = val % 10;
      val = val / 10;
   }
   while (p != buffer) {
      putchar('0' + *(--p));
   }
}

/* Helper: Print integer in Hexadecimal 
 */
static void print_hex_digits(unsigned int val, int nbdigits) {
   for (int i = (4*nbdigits)-4; i >= 0; i -= 4) {
      putchar("0123456789ABCDEF"[(val >> i) % 16]);
   }
}

static void print_hex(unsigned int val) {
   print_hex_digits(val, 8);
}

/* Main uprint Implementation 
 * Supports: %s (string), %x (hex), %d (decimal), %c (char)
 */
static int uprint(const char *fmt,...)
{
    va_list ap;

    for(va_start(ap, fmt);*fmt;fmt++)
    {
        if(*fmt=='%')
        {
            fmt++;
                 if(*fmt=='s') print_string(va_arg(ap,char *));
            else if(*fmt=='x') print_hex(va_arg(ap,int));
            else if(*fmt=='d') print_dec(va_arg(ap,int));
            else if(*fmt=='c') putchar(va_arg(ap,int));	   
            else putchar(*fmt);
        }
        else putchar(*fmt);
    }

    va_end(ap);

    return 0;
}

#endif // UART_H
