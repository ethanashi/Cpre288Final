#ifndef UART_H_
#define UART_H_

extern volatile char flag;
extern volatile char uart_data;
#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
// Function prototypes
void uart_init(void);
void uart_sendChar(char data);
char uart_receive(void);
void uart_sendStr(const char *data);

// (Optional) Interrupt-related functions if you plan to implement them later
void uart_interrupt_init(void);
void UART1_Handler(void);

#endif /* UART_H_ */
