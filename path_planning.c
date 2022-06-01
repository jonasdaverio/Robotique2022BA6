#include <ch.h>
#include <arm_math.h>
#include <motors.h>

#include "macros.h"
#include "main.h"
#include "path_planning.h"
#include "obstacles.h"

#define MOTOR_SPEED_CRUISE 700
#define PAUSE 1100 //pause between turns of 1s
#define PERIOD 100 //thread period of 100ms
#define DIST_OBJ 0.05 //min distance before turning, in m
#define TURN_SPEED 100

static THD_WORKING_AREA(slope_thd_wa, 128);
static THD_WORKING_AREA(path_planning_thd_wa, 256);

static THD_FUNCTION(path_planning_thd, arg);
static THD_FUNCTION(slope_thd, arg);

//i=0 -> tourne droite, 1 ->gauche
void turn(float angle)
{
	bool negative = angle < 0;
	if(negative) angle *= -1;

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	//place la position des roues du robot à (0;0)
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	if(negative)//TOURNE A DROITE
	{
		right_motor_set_speed(-TURN_SPEED);
		left_motor_set_speed(TURN_SPEED);
		while(left_motor_get_pos() < MTOSTEP(PI*WHEEL_TRACK*angle/360))
		{
			chThdSleepMilliseconds(10);
		}
	}
	else//TOURNE A GAUCHE
	{
		right_motor_set_speed(TURN_SPEED);
		left_motor_set_speed(-TURN_SPEED);
		while(right_motor_get_pos() < MTOSTEP(PI*WHEEL_TRACK*angle/360))
		{
			chThdSleepMilliseconds(10);
		}
	}
	right_motor_set_speed(MOTOR_SPEED_CRUISE);
	left_motor_set_speed(MOTOR_SPEED_CRUISE);
}

void init_path_planning(uint8_t mode)
{
	motors_init();
	//even mode means no turn (because accelerometer is currently a bit
	//unstable with rotations)
	if(mode%2)
	{
		right_motor_set_speed(MOTOR_SPEED_CRUISE);
		left_motor_set_speed(MOTOR_SPEED_CRUISE);
		chThdCreateStatic(slope_thd_wa, sizeof(slope_thd_wa),
				NORMALPRIO, slope_thd, NULL);
	}
	else
	{
		chThdCreateStatic(path_planning_thd_wa, sizeof(path_planning_thd_wa),
			NORMALPRIO, path_planning_thd, NULL);
	}
}

static THD_FUNCTION(slope_thd, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	wait_first_obstacle();
	while(chThdShouldTerminateX() == false)
	{
		systime_t time = chVTGetSystemTime();

		float distance_TOF = get_obstacles()->front;

		bool collision = (distance_TOF < DIST_OBJ)
		       	      || get_obstacles()->frontLeft1
		       	      || get_obstacles()->frontLeft2
		       	      || get_obstacles()->frontRight1
		       	      || get_obstacles()->frontRight2;
		if(collision)
		{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			break;
		}

		chThdSleepUntilWindowed(time, time+MS2ST(PERIOD));
	}
}



static THD_FUNCTION(path_planning_thd, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	uint8_t p = 0;
	wait_first_obstacle(); //We wait the first time but not anymore afterwards
	while(chThdShouldTerminateX() == false)
	{
		systime_t time = chVTGetSystemTime();

		float distance_TOF = get_obstacles()->front;

		bool collision = (distance_TOF < DIST_OBJ)
		       	      || get_obstacles()->frontLeft1
		       	      || get_obstacles()->frontLeft2
		       	      || get_obstacles()->frontRight1
		       	      || get_obstacles()->frontRight2;

		if(!collision)
		{
			right_motor_set_speed(MOTOR_SPEED_CRUISE);
			left_motor_set_speed(MOTOR_SPEED_CRUISE);
			chThdSleepUntilWindowed(time, time + MS2ST(PERIOD));
		}
		else
		{
			if (p%2==0)
			{
				turn(-90);//90° right
				distance_TOF = get_obstacles()->front;
				if(distance_TOF >= DIST_OBJ)
				{
					chThdSleepMilliseconds(PAUSE);
					turn(-90);
				}
				else
				{
					turn(-90);
				}
				p++;
			}
			else
			{
				turn(90);//90° left
				distance_TOF = get_obstacles()->front;
				if(distance_TOF >= DIST_OBJ)
				{
					chThdSleepMilliseconds(PAUSE);
					turn(90);
				}
				else
				{
					turn(90);
				}
				p++;
			}
		}
	}
}
