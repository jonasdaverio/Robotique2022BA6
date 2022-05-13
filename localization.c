#include <string.h>

#include <main.h>

#include <arm_math.h>
#include <chprintf.h>
#include <leds.h>
#include <motors.h>
#include <sensors/imu.h>
#include <wchar.h>

#include "localization.h"
#include "macros.h"
#include "my_math.h"

#define SPATIAL_DIMENSIONS 3
#define ANGLE_DIMENSION 3

//in m relative to beginning position
static float position[SPATIAL_DIMENSIONS] = {0};
//in m/s relative to beginning speed (should be zero relative to earth)
static float speed[SPATIAL_DIMENSIONS] = {0};

//matrix representation of the orientation (tradeoff between simple code
//and faster code: I could have used quaternions
//but I'v never really understood them)
static float32_t orientation_matrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};

static void double_integrate_acc(const float* measuredAcc, float delta_t);
static void integrate_gyro(const float* measuredGyroRate, float delta_t);
static void integrate_motors(float delta_t);

void reset_speed() { speed[0] = speed[1] = speed[2] = 0;}
void reset_position() { position[0] = position[1] = position[2] = 0;}
/* void reset_orientation(); */
const float* get_speed() { return speed; }
const float* get_position() { return position; }
const float* get_orientation() { return orientation_matrix; }

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
		static bool first_time = true;
		static time_t time = 0;

		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		//If it's the first time running the loop, don't do anything
		//Or delta_t wil be wrong
		if(first_time)
		{
			time = chVTGetSystemTime();
			first_time = false;
		}
		else
		{
			time_t prev_time = time;
			time = chVTGetSystemTime();
			float delta_t = (float)(time - prev_time)/CH_CFG_ST_FREQUENCY;

			//We integrate angular velocity
			/* integrate_gyro(imu_values.gyro_rate, delta_t); */

			//We integrate acceleration
			/* double_integrate_acc(imu_values.acceleration, delta_t); */

			integrate_motors(delta_t);
		}
	}
}

//Rotate the orientation matrix according to the rotation matrix
//rotation_matrix is in the robot reference frame
//orientation matrix is the world reference frame
static void rotate_orientation(float* rotation_matrix)
{
	arm_matrix_instance_f32 arm_orientation_matrix =
			{SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS, orientation_matrix};
	arm_matrix_instance_f32 arm_rotation_matrix = {SPATIAL_DIMENSIONS,
		SPATIAL_DIMENSIONS, rotation_matrix};

	//We then need to do a change of reference frame from the robot to the
	//world frame: rot' = orient*rot*orient^T
	//But as we then need to compute rot'*orient, we can directly simplify
	//everything to newOrient = orient*rot
	arm_mat_mult_f32(&arm_orientation_matrix, &arm_rotation_matrix, &arm_orientation_matrix);

	//We then need to normalize it, or it would otherwise grow very quickly
	normalize_matrix33(orientation_matrix); //Custom function
}

//small angle approximation of a rotation_matrix
static void init_rotation_matrix(float x_angle, float y_angle, float z_angle, float* rotation_matrix)
{
	rotation_matrix[0] =        1; rotation_matrix[1] = -z_angle; rotation_matrix[2] =  y_angle;
	rotation_matrix[3] =  z_angle; rotation_matrix[4] =        1; rotation_matrix[5] = -x_angle;
	rotation_matrix[6] = -y_angle; rotation_matrix[7] =  x_angle; rotation_matrix[8] =        1;
}

//Integrate motor speed to get position and orientation
static void integrate_motors(float delta_t)
{
	float speed_l = STEPTOM(left_motor_get_desired_speed());
	float speed_r = STEPTOM(right_motor_get_desired_speed());

	//We keep a copy for trapezoidal integration
	static float prev_delta_angle = 0;
	//A positive angle mean turning to the left
	float delta_angle = delta_t * (speed_r - speed_l)/WHEEL_TRACK;
	float rotation_matrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS];
	//The average is there for trapezoidal integration
	//We'll assume zero vertical speed
	init_rotation_matrix(0, 0, 0.5f*(prev_delta_angle+delta_angle), rotation_matrix);
	rotate_orientation(rotation_matrix);
	prev_delta_angle = delta_angle;

	//We keep a copy for trapezoidal integration
	static float prev_displacement_world[SPATIAL_DIMENSIONS] = {0};
	//The displacement is always in y in the robot reference frame
	float displacement_robot[SPATIAL_DIMENSIONS] = {0, delta_t*0.5f*(speed_l+speed_r), 0};
	float displacement_world[SPATIAL_DIMENSIONS] = {0};
	arm_matrix_instance_f32 arm_displacement_robot = {SPATIAL_DIMENSIONS, 1, displacement_robot};
	arm_matrix_instance_f32 arm_displacement_world = {SPATIAL_DIMENSIONS, 1, displacement_world};
	arm_matrix_instance_f32 arm_orientation_matrix = {SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS, orientation_matrix};
	//We compute the displacement in the world reference frame
	arm_mat_mult_f32(&arm_orientation_matrix, &arm_displacement_robot, &arm_displacement_world);

	//We finally update the position of the robot
	position[0] += 0.5f * (prev_displacement_world[0] + displacement_world[0]);
	position[1] += 0.5f * (prev_displacement_world[1] + displacement_world[1]);
	position[2] += 0.5f * (prev_displacement_world[2] + displacement_world[2]);

	memcpy(prev_displacement_world, displacement_world, SPATIAL_DIMENSIONS*sizeof(float));
}

//Trapezoidally integrate angular velocity, tricky because of the change
//of basis between the robot and the world frame
static void integrate_gyro(const float* measuredGyroRate, float delta_t)
{
	//We need to keep a copy for trapezoidal integration
	static float gyro_rate[ANGLE_DIMENSION] = {0};

	//delta_* means delta_angle, the small angular change between t and t+dt
	//We do a trapezoidal integration, so we take the average of consecutive
	//measurements
	float delta_x = 0.5f * delta_t * (gyro_rate[0] + measuredGyroRate[0]);
	float delta_y = 0.5f * delta_t * (gyro_rate[1] + measuredGyroRate[1]);
	float delta_z = 0.5f * delta_t * (gyro_rate[2] + measuredGyroRate[2]);
	gyro_rate[0] = measuredGyroRate[0];
	gyro_rate[1] = measuredGyroRate[1];
	gyro_rate[2] = measuredGyroRate[2];

	float rotation_matrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};
	init_rotation_matrix(delta_x, delta_y, delta_z, rotation_matrix);
	rotate_orientation(rotation_matrix);
}

//Trapezoidally double integrate acceleration into speed
static void double_integrate_acc(const float* measuredAcc, float delta_t)
{
	//We need to keep a copy for trapezoidal integration
	static float acc[SPATIAL_DIMENSIONS] = {0}; // in m/s^2

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
	arm_matrix_instance_f32 delta_v_world_arm_vec = {SPATIAL_DIMENSIONS, 1, delta_v_world};
	arm_matrix_instance_f32 arm_orientation_matrix = {SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS, orientation_matrix};
	arm_mat_mult_f32(&arm_orientation_matrix, &delta_v_arm_vec, &delta_v_world_arm_vec);
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
	reset_matrix(orientation_matrix);
}

static void reset_matrix(float32_t* matrix_data)
{
	for(int i=0; i<SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS; i++)
	{
		matrix_data[i] = (i%(SPATIAL_DIMENSIONS+1) == 0);
	}
}

