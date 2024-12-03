/*
 * movement.h
 *
 *  Created on: Sep 10, 2024
 *      Author: tlenberg
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "Timer.h"
#include <inc/tm4c123gh6pm.h>
#include "lcd.h"
#include "open_interface.h"

///moves the robot forward using the given amount
void move_forward(oi_t *sensor, int centimeters);

///moves the robot backward using the given amount
void move_backward(oi_t *sensor, int centimeters);

///turns the robot clockwise by the given degree
void turn_clockwise(oi_t *sensor, int degrees);

///turns the robot counterclockwise by the given degree
void turn_counter(oi_t *sensor, int degrees);

#endif /* MOVEMENT_H_ */
