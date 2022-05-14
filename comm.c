#include <main.h> 
#include <chprintf.h>
#include <serial_comm.h>

#include "comm.h"
#include "obstacles.h"

static THD_WORKING_AREA(comm_thd_wa, 512);
static THD_FUNCTION(comm_thd, arg);

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
		systime_t time = chVTGetSystemTime();

		binary_semaphore_t* obstacle_sem = get_obstacle_sem();
		chBSemWait(obstacle_sem);

		const obstacle_t* obstacles = get_obstacles();

		chprintf((BaseSequentialStream*)&SD3,
				"%f\n\r%d\n\r%d\n\r%d\n\r%d\n\r%d\n\r%d\n\r%d\n\r%d\n\r\n\r",
				obstacles->front, obstacles->frontLeft1, obstacles->frontRight1,
				obstacles->frontLeft2, obstacles->frontRight2, obstacles->left,
				obstacles->right, obstacles->rearLeft, obstacles->rearRight);
		chThdSleepUntilWindowed(time, time + MS2ST(100));
	}
}
