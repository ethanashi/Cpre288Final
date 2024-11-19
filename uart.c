#include "uart.h"
#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

volatile char flag = 0;
volatile char uart_data = 0;

void uart_init(void) {
    // Enable the clock to UART1 and GPIO Port B
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;  // UART1 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;  // GPIO Port B clock

    // Wait for the peripherals to be ready
    while ((SYSCTL_PRUART_R & SYSCTL_PRUART_R1) == 0) {};
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {};

    // Configure GPIO Port B pins 0 and 1 for UART1
    GPIO_PORTB_AFSEL_R |= 0x03;    // Enable alternate functions on PB0 and PB1
    GPIO_PORTB_PCTL_R &= ~0x000000FF;  // Clear PCTL for PB0 and PB1
    GPIO_PORTB_PCTL_R |= 0x00000011;   // Set PCTL for UART1 on PB0 and PB1
    GPIO_PORTB_DEN_R |= 0x03;          // Enable digital functionality on PB0 and PB1
    GPIO_PORTB_DIR_R |= 0x02;          // Set PB1 (TX) as output
    GPIO_PORTB_DIR_R &= ~0x01;         // Set PB0 (RX) as input

    // Disable UART1 before configuration
    UART1_CTL_R &= ~UART_CTL_UARTEN;

    // Calculate the baud rate divisors
    // For 115200 baud rate with a 16 MHz system clock
    uint16_t iBRD = 8;   // Integer part
    uint16_t fBRD = 44;  // Fractional part

    // Set the baud rate
    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;

    // Set line control parameters (8 data bits, no parity, one stop bit, no FIFO)
    UART1_LCRH_R = UART_LCRH_WLEN_8;

    // Use system clock as the UART clock source
    UART1_CC_R = UART_CC_CS_SYSCLK;

    // Enable UART1, TXE, and RXE
    UART1_CTL_R |= UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
}
//
void uart_interrupt_init(void) {
    // Enable UART1 receive interrupt
    UART1_IM_R |= UART_IM_RXIM;     // Enable interrupt on receive - page 924 of the datasheet

    // Clear any prior interrupts
    UART1_ICR_R |= UART_ICR_RXIC;   // Clear the receive interrupt flag

    // Set the priority of UART1 interrupt in the NVIC (optional)
    // UART1 interrupt is interrupt number 6 (see datasheet)
    // Bits 15-13 in NVIC_PRI1_R correspond to UART1
    // Lower numbers have higher priority
    // NVIC_PRI1_R = (NVIC_PRI1_R & 0xFFFF1FFF) | (priority << 13);  // Uncomment and set 'priority' if needed

    // Enable UART1 interrupt in NVIC
    NVIC_EN0_R |= (1 << 6);         // Enable interrupt number 6 (UART1) in NVIC

    // Optionally, register the ISR if using a function to vector table mapping
    // This depends on your compiler and startup code
    // If needed, you can register the ISR like this:
    IntRegister(INT_UART1, UART1_Handler);
}


void uart_sendChar(char data) {
    // Wait until the transmit FIFO is not full
    while (UART1_FR_R & UART_FR_TXFF) {};
    // Write the data to the data register
    UART1_DR_R = data;
}

char uart_receive(void) {
    // Wait until the receive FIFO is not empty
    while (UART1_FR_R & UART_FR_RXFE) {};
    // Read the data from the data register
    char data = (char)(UART1_DR_R & 0xFF);
    return data;
}

void uart_sendStr(const char *data) {
    while (*data != '\0') {
        uart_sendChar(*data++);
    }
}

//void uart_interrupt_init(void) {
//    // Enable interrupts for receiving bytes through UART1
//    UART1_IM_R |= UART_IM_RXIM; // Enable interrupt on receive - page 924
//
//    // Enable UART1 interrupt in NVIC
//    // UART1 is interrupt number 6 (Table 2-9 in the datasheet)
//    NVIC_EN0_R |= (1 << 6); // Enable UART1 interrupts - page 104
//
//    // Register the interrupt handler for UART1
//    // UART1 has vector number 22 (Table 2-9), but we use INT_UART1 constant
//    IntRegister(INT_UART1, UART1_Handler); // Provide the address of our interrupt handler
//}

void UART1_Handler(void) {
    // Check if the interrupt is due to received data
    if (UART1_MIS_R & UART_MIS_RXMIS) {
        flag = 1;
        // Read the received data
        uart_data = UART1_DR_R & 0xFF;

        // (Optional) Handle any errors (e.g., overrun, framing errors)

        // Clear the interrupt flag by writing to the interrupt clear register
        UART1_ICR_R |= UART_ICR_RXIC;

        // Process the received data
        // For example, store it in a buffer or set a flag
        // Ensure that shared variables are declared as volatile if accessed in both ISR and main
    }
}

int uart_receive_nonblocking(char *data) {
    // Check if the receive FIFO is empty
    if ((UART1_FR_R & UART_FR_RXFE) == 0) {
        // Data is available
        *data = (char)(UART1_DR_R & 0xFF);
        return 1; // Success
    } else {
        // No data available
        return 0; // No data received
    }
}

