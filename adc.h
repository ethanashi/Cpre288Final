/*
 * adc.h
 *
 *  Created on: Oct 28, 2024
 *      Author: ethana1
 */

#ifndef ADC_H_
#define ADC_H_

void adc_init(void);
int adc_read(void);
int adc_read_avg(void); // Updated to reflect averaging
float calculate_distance(void);

#endif /* ADC_H_ */
