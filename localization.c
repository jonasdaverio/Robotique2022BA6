#include <string.h>

#include <main.h>
#include <chprintf.h>

#include <arm_math.h>
#include <leds.h>
#include <motors.h>
#include <sensors/imu.h>

#include "localization.h"
#include "macros.h"
#include "my_math.h"

//in m relative to beginning position
static float position[SPATIAL_DIMENSIONS] = {0};
//in m/s relative to beginning speed (should be zero relative to earth)
static float speed[SPATIAL_DIMENSIONS] = {0};

//matrix representation of the orientation (tradeoff between simple code
//and faster code: I could have used quaternions
//but I'v never really understood them)
static float32_t orientation_matrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};

#ifdef INTEGRATE_ACC
static void double_integrate_acc(const float* measuredAcc, float delta_t);
#endif //INTEGRATE_ACC

static void integrate_motors(float delta_t);
static void integrate_gyro(const float* measuredGyroRate, float* rotation_matrix, float delta_t);
static void compute_rotation(const float* measuredGyroRate, const float* measuredAcc, float delta_t);

#ifdef USE_SENSOR_FUSION
static float* get_averaged_acceleration(const float* measuredAcc);
static float acc_buffer[ACC_BUFFER_SIZE*SPATIAL_DIMENSIONS] = {0};
#endif

const float* get_speed() { return speed; }
const float* get_position() { return position; }
const float* get_orientation() { return orientation_matrix; }


static void reset_matrix(float32_t* matrix_data);

static THD_WORKING_AREA(localization_thd_wa, 1024);
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

#ifdef INTEGRATE_ACC //Doesn't work well
			//We integrate acceleration
			double_integrate_acc(imu_values.acceleration, delta_t);
#else
			compute_rotation(imu_values.gyro_rate, imu_values.acceleration, delta_t);
			integrate_motors(delta_t); //Directly edit position and orientation
#endif //INTEGRATE_ACC
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

//Use both acc and gyro to determine non robot-Z rotation
static void compute_rotation(const float* measuredGyroRate, const float* measuredAcc, float delta_t)
{
	//We integrate angular velocity
	float gyro_rotation_matrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};
	integrate_gyro(measuredGyroRate, gyro_rotation_matrix, delta_t);
	float gyroOrientationMatrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};
	rotate_orientation(gyro_rotation_matrix, gyroOrientationMatrix);
	
#ifdef USE_SENSOR_FUSION //Not yet implemented
	float* averaged_acc_robot = get_averaged_acceleration(measuredAcc);
	
	//Because of drift, we need to correct the gyroscope measurement with gravity
	//We'll see how much we diverged from reality
	//And we'll compute a correction matrix to realign the z axis with gravity
	//Gravity doesn't provide any information about rotation around robot-Z axis

	arm_matrix_instance_f32 arm_average_acc_robot = {SPATIAL_DIMENSIONS, 1, averaged_acc_robot};
	arm_matrix_instance_f32 arm_orientation_matrix = {SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS,
														orientation_matrix};
	//We convert it in the world reference frame
	float averaged_acc_world[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};
	arm_matrix_instance_f32 arm_averaged_acc_world = {SPATIAL_DIMENSIONS, 1, averaged_acc_world};
	arm_mat_mult_f32(&arm_orientation_matrix, &arm_average_acc_robot, &arm_averaged_acc_world);

	chprintf((BaseSequentialStream*)&SD3, "%f\t%f\t%f\r\n",averaged_acc_robot[0], averaged_acc_robot[1], averaged_acc_robot[2]);

	//We first find the angle between averaged_acc_world and -z_angle, by linearizing
	//The angle is roughly equal to du/z, with du = sqrt(dx^2 + dy^2)
	//We go in the basis formed by -z, the projection of the measure
	//on the x-y plane, and the cross product of the two last vectors
	//And finally, we get the total matrix by PAP^T, with P the change of basis
	//and A the matrix of rotation correction. All operations are hardcoded to avoid
	//many matrix multiplicatoin
	float inv_sqr = fast_inverse_square_root(averaged_acc_world[0]*averaged_acc_world[0]
											+averaged_acc_world[1]*averaged_acc_world[1]);
	float coeff1 = averaged_acc_world[0]*averaged_acc_world[2]*inv_sqr;
	float coeff2 = averaged_acc_world[1]*averaged_acc_world[2]*inv_sqr;
	float acc_rotation_matrix[] = {1, 0, coeff1,
							  0, 1, coeff2,
						  -coeff1, -coeff2, 1};

	float accOrientationMatrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};
	rotate_orientation(acc_rotation_matrix, accOrientationMatrix);
	//We'll then compare the two result
	//The bigger the difference, the more we'll privilege the accelerometer reading
	//and vice versa
	
	//One way to check is to multiply one with the transpose of the other
	//If they were identical, we would get the identity
	arm_matrix_instance_f32 arm_gyro_orientation_matrix = {SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS,
															gyroOrientationMatrix};
	arm_matrix_instance_f32 arm_acc_orientation_matrix = {SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS,
															accOrientationMatrix};
	arm_mat_trans_f32(&arm_acc_orientation_matrix, &arm_acc_orientation_matrix);
	
	float deltaMatrix[SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS] = {0};
	arm_matrix_instance_f32 arm_deltaMatrix = {SPATIAL_DIMENSIONS, SPATIAL_DIMENSIONS, deltaMatrix};

	arm_mat_mult_f32(&arm_acc_orientation_matrix, &arm_gyro_orientation_matrix, &arm_deltaMatrix);
#else
	(void) measuredAcc;
#endif //USE_SENSOR_FUSION
	
	memcpy(orientation_matrix, gyroOrientationMatrix, SPATIAL_DIMENSIONS*SPATIAL_DIMENSIONS*sizeof(float));
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
	rotate_orientation(rotation_matrix, orientation_matrix);
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
static void integrate_gyro(const float* measuredGyroRate, float* rotation_matrix, float delta_t)
{
	//We need to keep a copy for trapezoidal integration
	static float gyro_rate[ANGLE_DIMENSION] = {0};

	//delta_* means delta_angle, the small angular change between t and t+dt
	//We do a trapezoidal integration, so we take the average of consecutive
	//measurements
	float delta_x = 0.5f * delta_t * (gyro_rate[0] + measuredGyroRate[0]);
	float delta_y = 0.5f * delta_t * (gyro_rate[1] + measuredGyroRate[1]);
	//We don't care about z as it is handled by the wheel
	gyro_rate[0] = measuredGyroRate[0];
	gyro_rate[1] = measuredGyroRate[1];

	init_rotation_matrix(delta_x, delta_y, 0, rotation_matrix);
}

#ifdef USE_SENSOR_FUSION
//Circular buffer implimentation with efficient average
static float* get_averaged_acceleration(const float* measuredAcc)
{
	static float* currentAcc = acc_buffer;
	static bool firstRound = true;
	static float meanAcc[SPATIAL_DIMENSIONS] = {0};
	
	if(firstRound)
	{
		uint8_t steps = (currentAcc - acc_buffer)/3;
		meanAcc[0] = (steps*meanAcc[0] + measuredAcc[0])/(steps+1);
		meanAcc[1] = (steps*meanAcc[1] + measuredAcc[1])/(steps+1);
		meanAcc[2] = (steps*meanAcc[2] + measuredAcc[2])/(steps+1);
		memcpy(currentAcc, measuredAcc, SPATIAL_DIMENSIONS*sizeof(float));
	}
	else
	{
		//As the number of elements will stay constant form now on,
		//we can compute the average by subtracting what goes out and
		//adding what goes in.
		meanAcc[0] += (measuredAcc[0] - currentAcc[0])/ACC_BUFFER_SIZE;
		meanAcc[1] += (measuredAcc[1] - currentAcc[1])/ACC_BUFFER_SIZE;
		meanAcc[2] += (measuredAcc[2] - currentAcc[2])/ACC_BUFFER_SIZE;
		memcpy(currentAcc, measuredAcc, SPATIAL_DIMENSIONS*sizeof(float));
	}

	currentAcc += SPATIAL_DIMENSIONS;
	if(currentAcc - acc_buffer >= SPATIAL_DIMENSIONS*ACC_BUFFER_SIZE)
	{
		firstRound = false;
		currentAcc = acc_buffer;
	}
	return meanAcc;
}
#endif

#ifdef INTEGRATE_ACC //Doesn't work well
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
	chprintf((BaseSequentialStream*)&SD3, "%f\r\n", position[0]);
}
#endif //INTEGRATE_ACC

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

