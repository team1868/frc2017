#ifndef SRC_PORTS2017_H_
#define SRC_PORTS2017_H_

/**
 * Ports
 */


/* ***************************** ROBOT PORTS **************************** */

/* ------------------- DRIVE TALON IDS ------------------ */
static const int LEFT_DRIVE_MASTER_ID					= 3;
static const int LEFT_DRIVE_SLAVE_ID 		 			= 2;
static const int RIGHT_DRIVE_MASTER_ID					= 4;
static const int RIGHT_DRIVE_SLAVE_ID 					= 1;

/* ---------------------- PWM PORTS --------------------- */
static const int FLYWHEEL_MOTOR_PWM_PORT				= 7;
static const int FEEDER_MOTOR_PWM_PORT					= 4;
static const int INTAKE_MOTOR_PWM_PORT					= 5;
static const int CLIMBER_MOTOR_PWM_PORT					= -1; //CHANGE THIS PORT

/* --------------------- PDP CHANNELS ------------------- */

/* ------------------ DIGITAL I/O PORTS ----------------- */
static const int INTAKE_ENCODER_A_PWM_PORT				= -1;
static const int INTAKE_ENCODER_B_PWM_PORT				= -1;

static const int FLYWHEEL_ENCODER_A_PWM_PORT			= 8;
static const int FLYWHEEL_ENCODER_B_PWM_PORT			= 9;

static const int DISTANCE_SENSOR_PWM_PORT				= -1;
/* ------------------ ANALOG IN PORTS --------------------*/

/* ------------------------ MISC -------------------------*/

/* ------------------- SOLENOID PORTS ------------------- */

/* ************************ DRIVER STATION PORTS ************************ */

/* ----------------- JOYSTICK USB PORTS ----------------- */
static const int LEFT_JOY_USB_PORT						= 0;
static const int RIGHT_JOY_USB_PORT						= 1;
static const int OPERATOR_JOY_USB_PORT					= 2;
static const int OPERATOR_JOY_B_USB_PORT				= 3;

/* -------------------- BUTTON PORTS -------------------- */

//Drive controller button ports
static const int DRIVE_DIRECTION_BUTTON_PORT			= 3;
static const int HIGH_LOW_GEAR_BUTTON_PORT				= 8;
static const int ARCADE_DRIVE_BUTTON_PORT				= 3;
static const int QUICK_TURN_BUTTON_PORT					= 1;
static const int BRAKE_BUTTON_PORT						= 2;

//Superstructure controller button ports
static const int FLYWHEEL_SWITCH_PORT					= 7; //Change port
static const int INTAKE_BUTTON_PORT						= 4; //Change port
static const int CLIMBER_BUTTON_PORT					= 5; //Change port

#endif /* SRC_PORTS2017_H_ */
