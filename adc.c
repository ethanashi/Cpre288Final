/*
 * adc.c
 *
 *  Created on: Oct 28, 2024
 *      Author: ethana1
 */



#include <stdint.h>
#include "adc.h"
#include <inc/tm4c123gh6pm.h>

#define NUM_SAMPLES 16

#define CAL_A 181.0
#define CAL_B -0.198
#define CAL_C 7.98e-5
#define CAL_D -1.09e-8

void adc_init(void) {
    SYSCTL_RCGCADC_R |= 0x1;
    SYSCTL_RCGCGPIO_R |= 0x2;
    while((SYSCTL_PRGPIO_R & 0x2) == 0) {};

    GPIO_PORTB_AFSEL_R |= 0x10;
    GPIO_PORTB_DEN_R &= ~0x10;
    GPIO_PORTB_AMSEL_R |= 0x10;

    ADC0_ACTSS_R &= ~0x8;
    ADC0_EMUX_R &= ~0xF000;
    ADC0_SSMUX3_R = 10;
    ADC0_SSCTL3_R = 0x6;
    ADC0_ACTSS_R |= 0x8;
}

// Read a sample from ADC0 Sequencer 3
int adc_read(void) {
    ADC0_PSSI_R = 0x8;
    while((ADC0_RIS_R & 0x8) == 0) {};
    int result = ADC0_SSFIFO3_R & 0xFFF;
    ADC0_ISC_R = 0x8;
    return result;
}

int adc_read_avg(void) {
    long sum = 0;
    int i = 0;
    for(i = 0; i < NUM_SAMPLES; i++) {
        ADC0_PSSI_R = 0x8;
        while((ADC0_RIS_R & 0x8) == 0) {};
        int result = ADC0_SSFIFO3_R & 0xFFF;
        ADC0_ISC_R = 0x8;
        sum += result;
    }
    return sum / NUM_SAMPLES;
}

float calculate_distance(void) {
    int adc_value = adc_read_avg();

    // Distance = CAL_A + CAL_B * x + CAL_C * x^2 + CAL_D * x^3
    float distance = CAL_A + (CAL_B * adc_value) + (CAL_C * adc_value * adc_value) + (CAL_D * adc_value * adc_value * adc_value);
    return distance;
}
