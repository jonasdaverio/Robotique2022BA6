#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//ChibiOS includes
#include <ch.h>
#include <hal.h>
#include <chprintf.h>

//e-puck_main-processor includes
#include <memory_protection.h>
#include <msgbus/messagebus.h>
#include <main.h>
#include <motors.h>

//Local includes 
#include "comm.h"
#include "localization.h"
#include "macros.h"
#include "obstacles.h"

messagebus_t bus; //declare in main.h, but defined here
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/* static THD_WORKING_AREA(comm_thd_wa, 256); */
/* static THD_FUNCTION(comm_thd, arg) */
/* { */
/* 	(void) arg; */
/* 	chRegSetThreadName(__FUNCTION__); */

/* 	while(chThdShouldTerminateX() == false) */
/* 	{ */
/* 		const float* position = get_position(); */
/* 		chprintf((BaseSequentialStream*) &SD3, "%f\t%f\t%f\r\n\r\n", */
/* 				position[0], position[1], position[2]); */

/* 		left_motor_set_speed(MTOSTEP(M_PI_4*WHEEL_TRACK)); */
/* 		right_motor_set_speed(-MTOSTEP(M_PI_4*WHEEL_TRACK)); */
/* 		chThdSleepSeconds(1); */

/* 		left_motor_set_speed(0); */
/* 		right_motor_set_speed(0); */
/* 		chThdSleepMilliseconds(500); */

/* 		left_motor_set_speed(MTOSTEP(0.05f)); */
/* 		right_motor_set_speed(MTOSTEP(0.05f)); */
/* 		chThdSleepSeconds(2); */

/* 		left_motor_set_speed(0); */
/* 		right_motor_set_speed(0); */
/* 		chThdSleepMilliseconds(500); */
/* 	} */
/* } */

int main(void)
{
	//general initialization
	halInit();
	chSysInit();
	mpu_init();

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	localization_init();
	obstacle_init();
	comm_init();
	motors_init();

	//Make the main thread sleep
	/* chThdCreateStatic(comm_thd_wa, sizeof(comm_thd_wa), */
	/* 		NORMALPRIO, comm_thd, NULL); */
	while(1)
	{
		chThdSleepSeconds(1);
	}

	return 0;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
