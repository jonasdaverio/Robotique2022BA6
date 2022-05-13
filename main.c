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
#include <serial_comm.h>

//Local includes 
#include "chthreads.h"
#include "localization.h"

messagebus_t bus; //declare in main.h, but defined here
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
	//general initialization
	halInit();
	chSysInit();
	mpu_init();

	messagebus_init(&bus, &bus_lock, &bus_condvar);
	serial_start();
	
	//Custom inits
	localization_init();

	//Make the main thread sleep
	uint8_t count = 0;
	while(1)
	{
		const float* position = get_position();
		const float* speed = get_speed();
		chprintf((BaseSequentialStream*)&SD3,
				"Position:\t%f\t%f\t%f\r\nSpeed:\t\t%f\t%f\t%f\r\n\r\n",
				position[0], position[1], position[2],
				speed[0], speed[1], speed[2]);
		count++;
		if(count == 10)
		{
			reset_orientation();
			reset_position();
			reset_speed();
			count = 0;
		}
		chThdSleepMilliseconds(700);
	}

	return 0;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
