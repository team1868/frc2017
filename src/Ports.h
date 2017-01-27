#ifndef SRC_PORTS_H_
#define SRC_PORTS_H_

/* ***************************** ROBOT PORTS **************************** */

/* ------------------- DRIVE TALON IDS ------------------ */
static const int LEFT_DRIVE_MASTER_ID 			= 5;
static const int LEFT_DRIVE_SLAVE_ID			= 4;
static const int RIGHT_DRIVE_MASTER_ID			= 2;
static const int RIGHT_DRIVE_SLAVE_ID			= 3;

/* --------------------- PWM PORTS ---------------------- */
static const int LEFT_DRIVE_MOTOR_A_PWM_PORT 			= 7;
static const int LEFT_DRIVE_MOTOR_B_PWM_PORT			= 8;
static const int RIGHT_DRIVE_MOTOR_A_PWM_PORT			= 2;
static const int RIGHT_DRIVE_MOTOR_B_PWM_PORT			= 1;

/* ----------------- DIGITAL I/O PORTS ------------------ */
static const int LEFT_ENCODER_A_PWM_PORT 				= 2;
static const int LEFT_ENCODER_B_PWM_PORT				= 3;
static const int RIGHT_ENCODER_A_PWM_PORT				= 0;
static const int RIGHT_ENCODER_B_PWM_PORT				= 1;

/* ------------------ ANALOG IN PORTS ------------------- */
static const int PRESSURE_SENSOR_PORT					= 3;

/* ----------------------- OTHER ------------------------ */
static const int COMPRESSOR_PORT						= 1;
static const int PNEUMATICS_CONTROL_MODULE_ID			= 1;

/* ------------------- SOLENOID PORTS ------------------- */
static const int GEAR_SHIFT_SOLENOID_PORT				= 4;

/* ************************ DRIVER STATION PORTS ************************ */

/* ----------------- JOYSTICK USB PORTS ----------------- */
static const int LEFT_JOY_USB_PORT						= 0;
static const int RIGHT_JOY_USB_PORT						= 1;
static const int OPERATOR_JOY_USB_PORT					= 2;
static const int OPERATOR_JOY_B_USB_PORT				= 3;

/* ----------------- DRIVE BUTTON PORTS ----------------- */
static const int DRIVE_DIRECTION_BUTTON_PORT			= 3;
static const int HIGH_LOW_GEAR_BUTTON_PORT				= 8;
static const int ARCADE_DRIVE_BUTTON_PORT				= 3;
static const int QUICK_TURN_BUTTON_PORT					= 1;
static const int DIAL_PIVOT_BUTTON_PORT					= 7;
static const int DIAL_PIVOT_SWITCH_PORT					= 2;
static const int BRAKE_BUTTON_PORT						= 2;

/* ------------- SUPERSTRUCTURE BUTTON PORTS ------------ */
// TODO


#endif /* SRC_PORTS_H_ */
