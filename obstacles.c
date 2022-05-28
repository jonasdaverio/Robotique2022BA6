#include <ch.h>

#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>

#include "msgbus/messagebus.h"
#include "obstacles.h"

#define PROX_THR 1000
#define MMTOM(n) 0.001f*n

extern messagebus_t bus;

static THD_WORKING_AREA(obstacle_thd_wa, 512);
static THD_FUNCTION(obstacle_thd, arg);

static BSEMAPHORE_DECL(obstacle_ready, true);
binary_semaphore_t* get_obstacle_sem() { return &obstacle_ready; }

static obstacle_t obstacles = {0};
const obstacle_t* get_obstacles() { return &obstacles; }

void obstacle_init()
{
	proximity_start();
	calibrate_ir();
	VL53L0X_start();

	chThdCreateStatic(obstacle_thd_wa, sizeof(obstacle_thd_wa),
			NORMALPRIO, obstacle_thd, NULL);
}

static THD_FUNCTION(obstacle_thd, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	messagebus_topic_t* prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");

	while(chThdShouldTerminateX() == false)
	{
		//We just need to wait, we don't need all the data on the bus
		messagebus_lock_acquire(prox_topic->lock);
		messagebus_condvar_wait(prox_topic->condvar);
		messagebus_lock_release(prox_topic->lock);

		obstacles.front       = MMTOM(VL53L0X_get_dist_mm());

		obstacles.frontRight1 = get_calibrated_prox(0) > PROX_THR;
		obstacles.frontRight2 = get_calibrated_prox(1) > PROX_THR;
		obstacles.right       = get_calibrated_prox(2) > PROX_THR;
		obstacles.rearRight   = get_calibrated_prox(3) > PROX_THR;
		obstacles.rearLeft    = get_calibrated_prox(4) > PROX_THR;
		obstacles.left        = get_calibrated_prox(5) > PROX_THR;
		obstacles.frontLeft2  = get_calibrated_prox(6) > PROX_THR;
		obstacles.frontLeft1  = get_calibrated_prox(7) > PROX_THR;

		chSysLock();
		chBSemSignalI(&obstacle_ready);
		chSysUnlock();
	}
}
