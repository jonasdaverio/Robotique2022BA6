#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <sensors/VL53L0X/VL53L0X.h>

int16_t pi_regulator_angle(float angle, float goal){

    float  error = 0;
    float  speed = 0;

    static float sum_error = 0;

    error = angle - goal;

    //disables the PI regulator if the error is to small
    //this avoids to always move as we cannot exactly be where we want and 
    //the camera is a bit noisy
    if(fabs(error) < ERROR_THRESHOLD/10){
        return 0;
    }

    sum_error += error;

    //we set a maximum and a minimum for the sum to avoid an uncontrolled growth
    if(sum_error > MAX_SUM_ERROR){
        sum_error = MAX_SUM_ERROR;
    }else if(sum_error < -MAX_SUM_ERROR){
        sum_error = -MAX_SUM_ERROR;
    }

    speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}
