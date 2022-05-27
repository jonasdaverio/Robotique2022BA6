#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <main.h>
#include <motors.h>

#include <leds.h>
#include <arm_math.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define SEND_FROM_MIC

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

   /* System init */
    halInit();
    chSysInit();
    mpu_init();
    //starts the serial communication
    serial_start();

    //inits the motors
    motors_init();

    //starts the TOF 
    VL53L0X_start();


    /* Infinite loop. */
    while (1) {
        //waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
