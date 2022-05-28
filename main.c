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

//Local includes 
#include "comm.h"
#include "localization.h"
#include "macros.h"
#include "obstacles.h"
#include "path_planning.h"
#include "selector.h"

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

	uint8_t mode = get_selector();

	comm_init();
	localization_init(mode);
	obstacle_init();
	init_path_planning(mode);

	//Make the main thread sleep
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
