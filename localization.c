#include <string.h>

#include <main.h>

#include <arm_math.h>
#include <chprintf.h>
#include <leds.h>
#include <sensors/imu.h>

#include "localization.h"
#include "my_math.h"

#define STANDARD_GRAVITY 9.80665f 
#define SPATIAL_DIMENSIONS 3
#define ANGLE_DIMENSION 3

//in m relative to beginning position
static float position[SPATIAL_DIMENSIONS] = {0};
//in m/s relative to beginning speed (should be zero relative to earth)
static float speed[SPATIAL_DIMENSIONS] = {0};

//matrix representation of the orientation (tradeoff between simple code
//and faster code: I could have used quaternions
//but I'v never really understood them)
static float32_t orientation_data[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};
static arm_matrix_instance_f32 orientation_matrix =
		{SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS, orientation_data};

static void double_integrate_acc(float* speed, float* acc,
		const float* measuredAcc, float delta_t);
static void integrate_gyro(float* gyro_rate, const float* measuredGyroRate,
		float delta_t);

void reset_speed() { speed[0] = speed[1] = speed[2] = 0;}
void reset_position() { position[0] = position[1] = position[2] = 0;}
/* void reset_orientation(); */
const float* get_speed() { return speed; }
const float* get_position() { return position; }

static void reset_matrix(float32_t* matrix_data); 

static THD_WORKING_AREA(localization_thd_wa, 512);
static THD_FUNCTION(localization_thd, arg);

void localization_init()
{
	set_body_led(1); //To signal IMU initialization and calibration
	chThdSleepSeconds(1); //Wait for the user to stop moving the robot
	imu_start();
	calibrate_acc();	
	calibrate_gyro();
	set_body_led(0); //Signals IMU init finished
	reset_orientation();

	chThdCreateStatic(localization_thd_wa, sizeof(localization_thd_wa),
			NORMALPRIO, localization_thd, NULL);
}

static THD_FUNCTION(localization_thd, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	messagebus_topic_t* imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	while(chThdShouldTerminateX() == false)
	{
		static time_t time = 0;
		time_t prev_time = time;

		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		time = chVTGetSystemTime();
		float delta_t = (float)(time - prev_time)/CH_CFG_ST_FREQUENCY;

		//We integrate angular velocity
		static float gyro_rate[ANGLE_DIMENSION] = {0};
		integrate_gyro(gyro_rate, imu_values.gyro_rate, delta_t);

		//We integrate acceleration
		static float acceleration[SPATIAL_DIMENSIONS] = {0}; // in m/s^2
		double_integrate_acc(speed, acceleration, imu_values.acceleration, delta_t);
	}
}

//Trapezoidally integrate angular velocity, tricky because of the change 
//of basis between the robot and the world frame
static void integrate_gyro(float* gyro_rate, const float* measuredGyroRate,
		float delta_t)
{
	//delta_* means delta_angle, the small angular change between t and t+dt
	//We do a trapezoidal integration, so we take the average of consecutive
	//measurements
	float delta_x = 0.5f * delta_t * (gyro_rate[0] + measuredGyroRate[0]);
	float delta_y = 0.5f * delta_t * (gyro_rate[1] + measuredGyroRate[1]);
	float delta_z = 0.5f * delta_t * (gyro_rate[2] + measuredGyroRate[2]);
	gyro_rate[0] = measuredGyroRate[0];
	gyro_rate[1] = measuredGyroRate[1];
	gyro_rate[2] = measuredGyroRate[2];

	//We first calculate the rotation matrix in the robot reference frame
	//As the sampling frequency is big relative to the angular velocity,
	//We can take the matrix to the first order, and everything simplifies
	float rotation_matrix_data[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] =
	{
		       1, -delta_z,  delta_y,
		 delta_z,        1, -delta_x,
		-delta_y,  delta_x,        1
	};
	arm_matrix_instance_f32 rotation_matrix = {SPATIAL_DIMENSIONS,
		SPATIAL_DIMENSIONS, rotation_matrix_data};

	//We then need to do a change of reference frame from the robot to the
	//world frame: rot' = orient*rot*orient^T
	//But as we then need to compute rot'*orient, we can directly simplify
	//everything to newOrient = orient*rot
	arm_mat_mult_f32(&orientation_matrix, &rotation_matrix, &orientation_matrix);
	
	//We then need to normalize it, or it would otherwise grow very quickly
	normalize_matrix33(orientation_data); //Custom function
}

//Trapezoidally double integrate acceleration into speed
//and update acc to measuredAcc
static void double_integrate_acc(float* speed, float* acc,
		const float* measuredAcc, float delta_t)
{
	//We first get the speed delta in the robot reference frame
	float delta_v[SPATIAL_DIMENSIONS] = {0};
	//The minus sign is there because the IMU measure the reaction acceleration
	delta_v[0] = -0.5f * (acc[0] + measuredAcc[0]);
	delta_v[1] = -0.5f * (acc[1] + measuredAcc[1]);
	delta_v[2] = -0.5f * (acc[2] + measuredAcc[2]);
	memcpy(acc, measuredAcc, 3*sizeof(float));

	//We then need to convert it back in the world reference frame
	arm_matrix_instance_f32 delta_v_arm_vec = {SPATIAL_DIMENSIONS, 1, delta_v};
	float delta_v_world[SPATIAL_DIMENSIONS] = {0};
	arm_matrix_instance_f32 delta_v_world_arm_vec = {SPATIAL_DIMENSIONS,
														1, delta_v_world};
	arm_mat_mult_f32(&orientation_matrix, &delta_v_arm_vec, &delta_v_world_arm_vec);
	//We can then subtract the gravity component
	delta_v_world[2] -= STANDARD_GRAVITY;

	float prevSpeed[SPATIAL_DIMENSIONS] = {0};
	memcpy(prevSpeed, speed, 3*sizeof(float));

	speed[0] += delta_t * delta_v_world[0];
	speed[1] += delta_t * delta_v_world[1];
	speed[2] += delta_t * delta_v_world[2];

	position[0] += 0.5f * delta_t*(speed[0] + prevSpeed[0]);
	position[1] += 0.5f * delta_t*(speed[1] + prevSpeed[1]);
	position[2] += 0.5f * delta_t*(speed[2] + prevSpeed[2]);
}	

void reset_orientation()
{
	reset_matrix(orientation_data);
}

static void reset_matrix(float32_t* matrix_data)
{
	for(int i=0; i<SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS; i++)
	{
		matrix_data[i] = (i%(SPATIAL_DIMENSIONS+1) == 0);
	}
}

