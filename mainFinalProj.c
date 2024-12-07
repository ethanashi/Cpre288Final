#include "math.h"
#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>

#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "adc.h"
#include "cyBot_Scan.h"
#include "movement.h"
#include "open_interface.h"

// Define constants for scanning and object detection
#define SCAN_ANGLE_STEP 2    // Step size for scanning in degrees
#define IR_THRESHOLD 50      // Distance threshold for detecting objects (cm)
#define TARGET_DISTANCE_CM 5 // Distance to maintain while approaching an object
#define OBJECT_WIDTH_THRESHOLD 10 // Minimum width to consider a parking space (degrees)

// Struct to hold object information (start angle, end angle, and distance)
typedef struct {
    int start_angle; // Starting angle of the object
    int end_angle;   // Ending angle of the object
    int distance;    // Distance to the object
} ObjectInfo;

// Array to store detected objects
ObjectInfo detected_objects[10];
int object_count = 0; // Count of detected objects

// Global variables for UART communication
volatile char uart_data;
volatile char flag = 0;

// Enum to manage different operational modes
typedef enum { MANUAL, AUTONOMOUS_SCAN, AUTONOMOUS_MOVE } Mode;
Mode current_mode = MANUAL;

// IR calibration data
float distances[] = {9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 50};
int ir_values[] = {1499, 1277, 1147, 1087, 987, 955, 919, 877, 842, 833, 803, 795, 779, 751, 731};

// Function to initialize all peripherals
void setup() {
    timer_init();               // Initialize the timer
    lcd_init();                 // Initialize the LCD
    uart_init(115200);          // Initialize UART with baud rate 115200
    uart_interrupt_init();      // Enable UART interrupt
    cyBOT_init_Scan(0b0001);    // Initialize CyBot scanning (servo only)
    adc_init();                 // Initialize ADC

    // Set calibration values for the servo motor
    right_calibration_value = 238000;  // Servo calibration for 0 degrees
    left_calibration_value = 1235500;  // Servo calibration for 180 degrees
}

// Function to calculate distance from IR sensor values using linear interpolation
int adc_distance() {
    int raw_ir = adc_read();  // Read raw IR sensor value
    if (raw_ir >= ir_values[0]) {
        return distances[0];  // If raw IR is beyond range, return closest distance
    }
    if (raw_ir <= ir_values[14]) {
        return distances[14]; // If raw IR is below range, return farthest distance
    }
    for (int i = 0; i < 14; i++) {
        // Linearly interpolate between two points to calculate distance
        if (raw_ir <= ir_values[i] && raw_ir > ir_values[i + 1]) {
            float slope = (distances[i + 1] - distances[i]) / (float)(ir_values[i + 1] - ir_values[i]);
            return distances[i] + slope * (raw_ir - ir_values[i]);
        }
    }
    return -1; // Return -1 if the calculation fails
}

// Function to perform a 180-degree IR scan and detect objects
void perform_180_ir_scan(cyBOT_Scan_t *scanData) {
    int angle;         // Angle of the servo motor during the scan
    int in_object = 0; // Flag to indicate if currently scanning an object
    int start_angle = -1; // Start angle of the object

    for (angle = 0; angle <= 180; angle += SCAN_ANGLE_STEP) {
        cyBOT_Scan(angle, scanData);   // Perform scan at the current angle
        int ir_distance = adc_distance(); // Convert raw IR to distance

        // Log scan data over UART
        char message[50];
        snprintf(message, sizeof(message), "Angle: %d\tIR Distance: %d cm\n", angle, ir_distance);
        uart_sendStr(message);

        // Detect objects based on distance threshold
        if (ir_distance < IR_THRESHOLD && !in_object) {
            // Start of a new object
            in_object = 1;
            start_angle = angle;
        } else if (ir_distance >= IR_THRESHOLD && in_object) {
            // End of the current object
            in_object = 0;
            detected_objects[object_count].start_angle = start_angle;
            detected_objects[object_count].end_angle = angle;
            detected_objects[object_count].distance = ir_distance;
            object_count++;
        }
    }

    // Display all detected objects
    uart_sendStr("\nDetected Objects:\n");
    uart_sendStr("Object\tStart Angle\tEnd Angle\n");
    uart_sendStr("--------------------------------\n");
    for (int i = 0; i < object_count; i++) {
        char message[50];
        snprintf(message, sizeof(message), "Object %d\t%d\t\t%d\n", i + 1, detected_objects[i].start_angle, detected_objects[i].end_angle);
        uart_sendStr(message);
    }
}

// Function to identify a parking spot based on 4 objects forming a consistent pattern
void find_parking_spot() {
    if (object_count < 4) {
        uart_sendStr("\nInsufficient objects to determine a parking spot.\n");
        return;
    }

    int start_index = -1; // Index of the first object in the parking spot
    for (int i = 0; i <= object_count - 4; i++) {
        int valid_spot = 1; // Assume a valid spot unless proven otherwise
        for (int j = i; j < i + 4; j++) {
            // Check if each object width exceeds the minimum threshold
            if ((detected_objects[j].end_angle - detected_objects[j].start_angle) < OBJECT_WIDTH_THRESHOLD) {
                valid_spot = 0; // Invalidate the spot if any object is too narrow
                break;
            }
        }
        if (valid_spot) {
            start_index = i; // Record the starting index of the valid spot
            break;
        }
    }

    if (start_index == -1) {
        uart_sendStr("\nNo valid parking spot found.\n");
    } else {
        char message[100];
        snprintf(message, sizeof(message), "\nParking spot found between objects %d and %d.\n", start_index + 1, start_index + 4);
        uart_sendStr(message);

        // Navigate to the identified parking spot
        // Additional movement logic can be implemented here
    }
}

int main(void) {
    setup(); // Initialize peripherals and hardware

    cyBOT_Scan_t scanData; // Object for scan data
    uart_sendStr("Press 's' to start scanning for parking spots.\n");

    while (1) {
        if (flag) {
            flag = 0;
            if (uart_data == 's') {
                object_count = 0; // Reset object count before scanning
                perform_180_ir_scan(&scanData); // Perform a scan
                find_parking_spot(); // Find a suitable parking spot
            } else if (uart_data == 'q') {
                uart_sendStr("Exiting program.\n");
                break;
            }
        }
    }
    return 0;
}
