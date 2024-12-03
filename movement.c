/*
 * movement.c
 *
 *  Created on: Sep 10, 2024
 *      Author: tlenberg
 */

#include <movement.h>

/*
 * \brief Moves the robot forward
 * \param centimeters, distance to move
 * \param *sensor
 */
void move_forward(oi_t *sensor, int centimeters) {
    //holds the distance to move in millimeters
    double distance = centimeters * 10;
    oi_setWheels(150, 150);

    //total distance moved so far
    double total = 0;
    double total2 = 0;
    //moves wheels forward by distance
    while (total < distance) {
        //updates the distance
        oi_update(sensor);
       /* If the robot  comes in contact with an object, the robot should attempt to go
           around the object by issuing the following commands: back up 15 cm, turn 90 degrees, move laterally 25cm,
           then turn 90 degrees forward*/
        if(sensor->bumpLeft || (sensor->bumpLeft && sensor->bumpRight)) {
            move_backward(sensor, 15);
            turn_clockwise(sensor, 75);
            oi_setWheels(150, 150);
            total2 = 0;
            while (total2 < 25 * 10) {
                oi_update(sensor);
                total2 += sensor->distance;
            }
            turn_counter(sensor, 75);
            oi_setWheels(150, 150);
        }

        else if (sensor->bumpRight) {
            total2 = 0;
            move_backward(sensor, 15);
            turn_counter(sensor, 75);
            oi_setWheels(150, 150);
            while (total2 < 25 * 10)
            {
                oi_update(sensor);
                total2 += sensor->distance;
            }
            turn_clockwise(sensor, 75);
            oi_setWheels(150, 150);
        }
        oi_update(sensor);
        total += sensor->distance;

    }

    //stop the robot
    oi_setWheels(0, 0);
    return;
}

/* \brief moves the robot backward
 * \param centimeters, distance to move the robot backward
 * \param *sensor
 */
void move_backward(oi_t *sensor, int centimeters) {
    //holds the distance to move in millimeters
    double distance = centimeters * -10;

    //sets the wheels
    oi_setWheels(-150, -150);

    //total distance moved so far
    double total3 = 0;

    //moves wheels backward by distance
    while (total3 > distance) {
       //updates the distance
       oi_update(sensor);
       total3 += sensor->distance;
    }

    //stops the robot
    oi_setWheels(0, 0);

    return;
}

/*
 * \brief turn the robot clockwise by given degree
 * \param degrees, degree to turn the robot clockwise
 * \parm *sensor
 */
void turn_clockwise(oi_t *sensor, int degrees) {
    //degrees = degrees * -1;
    //turn the wheels to the left
    oi_setWheels(-150, 150);

    //total value the robot has turned
    int total = 0;
    while (total > degrees) {
        oi_update(sensor);
        total += sensor->angle;
    }

    //stop robot and return
    oi_setWheels(0, 0);

    return;
}

/*
 * \brief turn the robot counter-clockwise by given degree
 * \param degrees, degrees to turn counter-clockwise
 * \param *sensor
 */
void turn_counter(oi_t *sensor, int degrees) {
    //turn wheels to the right
    oi_setWheels(150, -150);

    //total value the robot has turned
    int total = 0;
    while (total < degrees) {
        oi_update(sensor);
        total += sensor->angle;
    }

    //stop robot and return
    oi_setWheels(0, 0);

    return;
}
