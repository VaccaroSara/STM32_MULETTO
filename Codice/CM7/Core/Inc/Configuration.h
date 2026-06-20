/*
 * Configuration.h
 *
 *
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "math.h"

//TRACTION PID
#define TRACTION_SAMPLING_TIME 0.01 //[s]
#define MAX_U_TRACTION 2.6 // [V]
#define MIN_U_TRACTION -2.6 // [V]
#define KP_TRACTION 0.006
#define KI_TRACTION 0.002

//STEERING PID
#define STEERING_SAMPLING_TIME 0.01 //[s]
#define MAX_U_STEERING 20 // [°]
#define MIN_U_STEERING -20 //[°]
#define KP_STEERING 120
#define KI_STEERING 60

#define ENCODER_SAMPLING_TIME 0.01 //[s]

//Vehicle parameters for RPM<->m/s
#define WHEEL_RADIUS 0.03 //m
#define MOTOR_REVOLUTION_FOR_ONE_WHEEL_REVOLUTION 5

#define MAX_CURVATURE_RADIUS_FOR_STRAIGHT 10


#endif /* INC_CONFIGURATION_H_ */
