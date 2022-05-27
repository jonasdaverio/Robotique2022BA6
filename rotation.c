#include <ch.h>
#include <motors.h>

#include "rotation.h"
#include "obstacles.h"

#define MIN_VALUE_THRESHOLD	10000 

#define PAUSE 			1000 //thread pause of 1000ms
#define DIST_OBJ 100 //en mm = 10cm
#define COEF 80 // nombre de 'step' par centimètre 

int p;
p=0;

void turn (double angle, bool i){ //I=0 -> tourne droite, 1 ->gauche
	

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	left_motor_set_pos(0); 
	right_motor_set_pos(0);//place la position des roues du robot à (0;0)
	if (i==0){
		right_motor_set_speed(-300); //TOURNE A DROITE
		left_motor_set_speed(300);
		while(left_motor_get_pos()<(COEF*(15.7*angle)/360)){ 
			chThdSleepMilliseconds(10);
		}
		 break;
	}
	else{	
		right_motor_set_speed(300);//TOURNE A GAUCHE
		left_motor_set_speed(-300);
		while(right_motor_get_pos()<(COEF*(15.7*angle)/360)){
			chThdSleepMilliseconds(10);
		}
		 break;
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	right_motor_set_speed(MOTOR_SPEED_LIMIT*0.8);
	left_motor_set_speed(MOTOR_SPEED_LIMIT*0.8);
}
        //time = chVTGetSystemTime(); 


        //MODIFIER
        //f_mode = get_mode();

        //distance_TOF is updated by VL53L0X TOF library //OUI
	distance_TOF = get_obstacles()->front;

        //computes the speed to give to the motors
        //speed = pi_regulator_distance((float)distance_TOF*CM, DIST_PLAY*CM);

        //float angle = get_angle();
        //computes a correction factor to let the robot rotate to be aligned with the sound source
        //speed_correction = pi_regulator_angle(angle, 0); 
While (1){
	if(distance_TOF >= DIST_OBJ){
		right_motor_set_speed(MOTOR_SPEED_LIMIT*0.8);
		left_motor_set_speed(MOTOR_SPEED_LIMIT*0.8);
	}
	else{
		if (p%2==0){
			turn(90,0);//fonction, arrête le robot, tourne de 90 degrés droite, robot reprend sa vitesse

			if(distance_TOF >= DIST_OBJ){
				chThdSleepMilliseconds(1000);//attend 1seconde (il avance pendant 1 seconde)
				turn(90,0);		
			}
			else {
				turn(90,0);
			}
		
		else {
			turn(90,1);//fonction, arrête le robot, tourne de 90 degrés gauche, robot reprend sa vitesse

			if(distance_TOF >= DIST_OBJ){
				chThdSleepMilliseconds(1000);
				turn(90,1);		
			}
			else {
				turn(90,1);
			}
		p++

		}
	}
}

	break;