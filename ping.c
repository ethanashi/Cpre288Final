#include "ping.h"
#include "timer.h"
#include "lcd.h"
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"
#include <inc/hw_ints.h>

#define DEBUG_LED_PIN 0x04
    int OVERFLOW = 0;



volatile uint32_t echo_pulse_width = 0;
volatile bool expecting_falling = false;

void Timer3B_Handler(void);

void debug_led_init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {};

    GPIO_PORTB_AFSEL_R &= ~DEBUG_LED_PIN;
    GPIO_PORTB_DEN_R |= DEBUG_LED_PIN;
    GPIO_PORTB_DIR_R |= DEBUG_LED_PIN;
    GPIO_PORTB_DATA_R &= ~DEBUG_LED_PIN;
}

void ping_trigger_init(void) {

    GPIO_PORTB_AFSEL_R &= ~0x08;
    GPIO_PORTB_PCTL_R &= ~0x0000F000;
    GPIO_PORTB_DEN_R |= 0x08;
    GPIO_PORTB_DIR_R |= 0x08;
    GPIO_PORTB_DATA_R &= ~0x08;
}

void ping_echo_init(void) {
    GPIO_PORTB_AFSEL_R |= 0x08;
    GPIO_PORTB_PCTL_R &= ~0x0000F000;
    GPIO_PORTB_PCTL_R |= 0x00007000;
    GPIO_PORTB_DEN_R |= 0x08;
    GPIO_PORTB_DIR_R &= ~0x08;

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
    while((SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R3) == 0) {};

    TIMER3_CTL_R &= ~TIMER_CTL_TBEN;
    TIMER3_CFG_R = TIMER_CFG_16_BIT;
    TIMER3_TBMR_R = TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCMR;

    TIMER3_CTL_R &= ~TIMER_CTL_TBEVENT_M;
    TIMER3_CTL_R |= TIMER_CTL_TBEVENT_BOTH;

    TIMER3_TBILR_R = 0xFFFF;
    TIMER3_TBPR_R = 0xFF;

    TIMER3_ICR_R = TIMER_ICR_CBECINT;

    TIMER3_IMR_R |= TIMER_IMR_CBEIM;

    IntRegister(INT_TIMER3B, Timer3B_Handler);
    IntEnable(INT_TIMER3B);

    TIMER3_CTL_R |= TIMER_CTL_TBEN;
}

void ping_init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {};

    ping_trigger_init();
    debug_led_init();

    IntMasterEnable();
}

int ping_read(void) {

    echo_pulse_width = 0;
    expecting_falling = false;

    ping_trigger_init();
    GPIO_PORTB_DATA_R &= ~0x08;
    timer_waitMicros(2);
    GPIO_PORTB_DATA_R |= 0x08;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R &= ~0x08;

    ping_echo_init();

    unsigned int timeout = 60000;
    while(echo_pulse_width == 0 && timeout > 0) {
        timer_waitMicros(1);
        timeout--;
    }

    if(timeout == 0) {

        echo_pulse_width = 0;
    }

    ping_trigger_init();

    return echo_pulse_width;
}

void Timer3B_Handler(void) {
    static uint32_t rising_edge_time = 0;

    if (TIMER3_MIS_R & TIMER_MIS_CBEMIS) {
        uint32_t current_prescale = TIMER3_TBPS_R & 0xFF;
        uint32_t current_time = TIMER3_TBR_R & 0xFFFF;

        uint32_t full_time = (current_prescale << 16) | current_time;

        if (!expecting_falling) {

            rising_edge_time = full_time;
            expecting_falling = true;

            GPIO_PORTB_DATA_R |= DEBUG_LED_PIN;
        } else {

            if (full_time <= rising_edge_time) {

                echo_pulse_width = rising_edge_time - full_time;
            } else {

                echo_pulse_width = (rising_edge_time + 0xFFFFFF) - full_time;
                OVERFLOW++;
            }
            expecting_falling = false;

            GPIO_PORTB_DATA_R &= ~DEBUG_LED_PIN;
        }

        TIMER3_ICR_R = TIMER_ICR_CBECINT;
    }
}
