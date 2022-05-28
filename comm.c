#include <string.h>

#include <ch.h>

#include <serial_comm.h>

#include "comm.h"
#include "localization.h"
#include "obstacles.h"

#define COMM_PERIOD 100 //ms

static THD_WORKING_AREA(comm_thd_wa, 512);
static THD_FUNCTION(comm_thd, arg);

//From TP4
static void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

void comm_init()
{
	serial_start();
	chThdCreateStatic(comm_thd_wa, sizeof(comm_thd_wa),
			NORMALPRIO, comm_thd, NULL);
}

static THD_FUNCTION(comm_thd, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while(chThdShouldTerminateX() == false)
	{
		wait_obstacle_ready();
		systime_t time = chVTGetSystemTime();

		const obstacle_t* obstacles = get_obstacles();
		const float* position = get_position();
		const float* orientation = get_orientation();

#define size_obstacles (sizeof(float) + sizeof(uint8_t))
#define size_position (SPATIAL_DIMENSIONS * sizeof(float))
#define size_orientation (SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS*sizeof(float))

		uint8_t bytes[size_position+size_orientation+size_obstacles] = {0};

		memcpy(bytes, position, size_position);
		memcpy(bytes + size_position, orientation, size_orientation);
		memcpy(bytes + size_position + size_orientation, &(obstacles->front), sizeof(float));
		//We'll pack the bools in a byte
		bytes[size_position+size_orientation+sizeof(float)] = obstacles->frontRight1
														+ (obstacles->frontRight2 << 1)
														+ (obstacles->right << 2)
														+ (obstacles->rearRight << 3)
														+ (obstacles->rearLeft << 4)
														+ (obstacles->left << 5)
														+ (obstacles->frontLeft2 << 6)
														+ (obstacles->frontLeft1 << 7);
		SendUint8ToComputer(bytes, sizeof(bytes));

		chThdSleepUntilWindowed(time, time + MS2ST(COMM_PERIOD));
	}
}
