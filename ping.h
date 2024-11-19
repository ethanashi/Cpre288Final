// ping.h
#ifndef PING_H_
#define PING_H_
extern int OVERFLOW;

#include <stdint.h>
// Initialize PING))) sensor for triggering
void ping_trigger_init(void);

// Initialize PING))) sensor for echo capture
void ping_echo_init(void);

// Initialize PING))) sensor (initially as trigger)
void ping_init(void);

// Read echo pulse width in clock cycles
int ping_read(void);

// Volatile variable to store pulse width
extern volatile uint32_t echo_pulse_width;

// Initialize debug LED
void debug_led_init(void);

#endif /* PING_H_ */
