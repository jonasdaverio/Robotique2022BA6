#include <string.h>

#include <arm_math.h>
#include <ch.h>
#include <msgbus/messagebus.h>

#include <leds.h>
#include <motors.h>
#include <sensors/imu.h>

#include "localization.h"
#include "macros.h"
#include "my_math.h"

#define EPSILON_ANGLE 0.0001f

extern messagebus_t bus;

//in m relative to beginning position
static float position[SPATIAL_DIMENSIONS] = {0};
//in m/s relative to beginning speed (should be zero relative to earth)
static float speed[SPATIAL_DIMENSIONS] = {0};

//matrix representation of the orientation (tradeoff between simple code
//and faster code: I could have used quaternions
//but I'v never really understood them)
static float32_t orientation_matrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};

static void integrate_motors(float delta_t);
static void compute_rotation(const float* measuredAcc);

static float acc_buffer[ACC_BUFFER_SIZE*SPATIAL_DIMENSIONS] = {0};
static float meanAcc[SPATIAL_DIMENSIONS] = {0};
static void fill_acc_buffer(void);
static void update_average_acc(const float* measuredAcc);

const float* get_speed() { return speed; }
const float* get_position() { return position; }
const float* get_orientation() { return orientation_matrix; }


static void reset_matrix(float32_t* matrix_data);

static THD_WORKING_AREA(localization_thd_wa, 1024);
static THD_FUNCTION(localization_thd, arg);

static uint8_t loc_mode = 0;

void localization_init(uint8_t mode)
{
	//odd mode means no accelerometer
	loc_mode = mode;
	set_body_led(1); //To signal IMU initialization and calibration
	chThdSleepSeconds(1); //Wait for the user to stop moving the robot
	imu_start();
	calibrate_acc();
	calibrate_gyro();
	fill_acc_buffer();
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

			integrate_motors(delta_t);
			//because of instability, we might want to disable it
			if(loc_mode%2)
			{
				compute_rotation(imu_values.acceleration);
			}
		}
	}
}

//Rotate the orientation matrix according to the rotation matrix
//rotation_matrix is in the robot reference frame
//orientation matrix is the world reference frame
static void rotate_orientation(float* rotation_matrix, float* orientation_matrix_out)
{
	arm_matrix_instance_f32 arm_orientation_matrix =
		{SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS, orientation_matrix};
	arm_matrix_instance_f32 arm_rotation_matrix = {SPATIAL_DIMENSIONS,
		SPATIAL_DIMENSIONS, rotation_matrix};
	arm_matrix_instance_f32 arm_orientation_matrix_out =
		{SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS, orientation_matrix_out};

	//We then need to do a change of reference frame from the robot to the
	//world frame: rot' = orient*rot*orient^T
	//But as we then need to compute rot'*orient, we can directly simplify
	//everything to newOrient = orient*rot
	arm_mat_mult_f32(&arm_orientation_matrix, &arm_rotation_matrix, &arm_orientation_matrix_out);

	//We then need to normalize it, or it would otherwise grow very quickly
	normalize_matrix33(orientation_matrix_out); //Custom function
}

//Use the accelerometer to determine non robot-Z rotation
static void compute_rotation(const float* measuredAcc)
{
	update_average_acc(measuredAcc);
	float* averaged_acc_robot = meanAcc;
	//We'll compare our orientation with gravity which by definition
	//should point in z. We'll see how much we diverged from reality
	//And we'll compute a correction matrix to realign the z axis with gravity
	//Gravity doesn't provide any information about rotation around robot-Z axis

	arm_matrix_instance_f32 arm_average_acc_robot = {SPATIAL_DIMENSIONS, 1, averaged_acc_robot};
	arm_matrix_instance_f32 arm_orientation_matrix = {SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS,
														orientation_matrix};
	//We convert it in the world reference frame
	float averaged_acc_world[SPATIAL_DIMENSIONS] = {0};
	arm_matrix_instance_f32 arm_averaged_acc_world = {SPATIAL_DIMENSIONS, 1, averaged_acc_world};
	arm_mat_mult_f32(&arm_orientation_matrix, &arm_average_acc_robot, &arm_averaged_acc_world);


	//We first find the angle between averaged_acc_world and -z, by linearizing.
	//This angle is roughly equal to sqrt(gx^2 + gy^2)/sqrt(gx^2 + gy^2 + gz^2)
	//With g being the acceleration vector.
	//We go in the basis formed by z, the projection of the measure
	//on the x-y plane, and the opposite of the cross product of the last two vectors.
	//And finally, we get the total matrix by PAP^T, with P the change of basis
	//and A the matrix of rotation correction. All operations are hardcoded to avoid
	//many matrix multiplicatoin
	float inv_sqr = fast_inverse_square_root(averaged_acc_world[0]*averaged_acc_world[0]
											+averaged_acc_world[1]*averaged_acc_world[1]
											+averaged_acc_world[2]*averaged_acc_world[2]);
	float coeff1 = averaged_acc_world[0]*inv_sqr;
	float coeff2 = averaged_acc_world[1]*inv_sqr;
	float acc_rotation_matrix[] = {1,      0, -coeff1,
								   0,      1, -coeff2,
							  coeff1, coeff2, 1};

	float accOrientationMatrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};
	rotate_orientation(acc_rotation_matrix, accOrientationMatrix);

	memcpy(orientation_matrix, accOrientationMatrix, SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS*sizeof(float));
}

//small angle approximation of a rotation_matrix
static void init_rotation_matrix(float x_angle, float y_angle, float z_angle, float* rotation_matrix)
{
	rotation_matrix[0] =        1; rotation_matrix[1] = -z_angle; rotation_matrix[2] =  y_angle;
	rotation_matrix[3] =  z_angle; rotation_matrix[4] =        1; rotation_matrix[5] = -x_angle;
	rotation_matrix[6] = -y_angle; rotation_matrix[7] =  x_angle; rotation_matrix[8] =        1;
}

//Integrate motor speed to get position and orientation
//return whether it's turning 
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
	rotate_orientation(rotation_matrix, orientation_matrix);
	prev_delta_angle = delta_angle;

	//We keep a copy for trapezoidal integration
	static float prev_displacement_world[SPATIAL_DIMENSIONS] = {0};
	//The displacement is always in -y in the robot reference frame
	float displacement_robot[SPATIAL_DIMENSIONS] = {0, -delta_t*0.5f*(speed_l+speed_r), 0};
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

static void fill_acc_buffer(void)
{
	messagebus_topic_t* imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	for(uint8_t i=0; i<ACC_BUFFER_SIZE; i++)
	{
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
		float* measuredAcc = imu_values.acceleration;
		float* currentAcc = acc_buffer + SPATIAL_DIMENSIONS*i;
		meanAcc[0] =  (i*meanAcc[0] + measuredAcc[0])/(i+1);
		meanAcc[1] =  (i*meanAcc[1] + measuredAcc[1])/(i+1);
		meanAcc[2] =  (i*meanAcc[2] + measuredAcc[2])/(i+1);
		memcpy(currentAcc, measuredAcc, SPATIAL_DIMENSIONS*sizeof(float));
	}
}

//Circular buffer implementation with efficient average
static void update_average_acc(const float* measuredAcc)
{
	static float* currentAcc = acc_buffer;

	//As the number of elements will stay constant form now on,
	//we can compute the average by subtracting what goes out and
	//adding what goes in.
	meanAcc[0] += (measuredAcc[0] - currentAcc[0])/ACC_BUFFER_SIZE;
	meanAcc[1] += (measuredAcc[1] - currentAcc[1])/ACC_BUFFER_SIZE;
	meanAcc[2] += (measuredAcc[2] - currentAcc[2])/ACC_BUFFER_SIZE;
	memcpy(currentAcc, measuredAcc, SPATIAL_DIMENSIONS*sizeof(float));

	currentAcc += SPATIAL_DIMENSIONS;
	if(currentAcc - acc_buffer >= SPATIAL_DIMENSIONS*ACC_BUFFER_SIZE)
	{
		currentAcc = acc_buffer;
	}
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

