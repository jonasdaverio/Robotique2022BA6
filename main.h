#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/* LEDs that can be used in EPUCK2
  LED1 			: GPIOD pin 5
  LED3 			: GPIOD pin 6
  LED5 			: GPIOD pin 10
  LED7 			: GPIOD pin 11
  FRONT_LED 	: GPIOD pin 14
WARNING : Not on the same port !!
  BODY_LED		: GPIOB pin 2
*/
//#define LED1     	GPIOD, 5
//#define LED3     	GPIOD, 6
//#define LED5     	GPIOD, 10
//#define LED7     	GPIOD, 11
//#define FRONT_LED	GPIOD, 14
//#define BODY_LED	GPIOB, 2

//constants for the differents parts of the project
#define ROTATION_THRESHOLD		50
#define ROTATION_COEFF			0.7f
#define	GOAL_ANGLE				0.0f
#define ERROR_THRESHOLD			0.1f	//MODIFY IF NOISE COMING FROM TOF
#define KP						160.0f
#define KI 						0.05f	//must not be zero
#define MAX_SUM_ERROR			(MOTOR_SPEED_LIMIT/10)
#define MOTOR_SPEED_CRUISE		500
#define DIST_TRESHOLD			20
#define DIST_TRESHOLD_H			(DIST_GOAL + DIST_TRESHOLD)
#define DIST_TRESHOLD_L			(DIST_GOAL - DIST_TRESHOLD)
#define PAUSE 			1000 //thread pause of 1000ms
#define DIST_OBJ 100 //en mm = 10cm
#define COEF 80 // nombre de 'step' par centimètre de la roue

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
