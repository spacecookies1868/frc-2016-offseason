#ifndef SRC_PORTS_H_
#define SRC_PORTS_H_

/* ***************************** ROBOT PORTS **************************** */

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
static const int BRAKE_SOLENOID_A_PORT					= 6;
static const int BRAKE_SOLENOID_B_PORT					= 5;
static const int INTAKE_SOLENOID_A_PORT					= 0;
static const int INTAKE_SOLENOID_B_PORT					= 1;
static const int DEFENSE_MANIP_SOLENOID_A_PORT			= 3;
static const int DEFENSE_MANIP_SOLENOID_B_PORT			= 2;

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

/* ------------ OTHER CONTROLLER BUTTON PORTS ----------- */
static const int DEFENSE_ID_1_BUTTON_PORT				= 3;
static const int DEFENSE_ID_2_BUTTON_PORT				= 4;
static const int DEFENSE_ID_3_BUTTON_PORT				= 5;
static const int STOP_AUTO_BUTTON_PORT					= 6;
static const int DEFENSE_POSITION_ID_1_BUTTON_PORT		= 1;
static const int DEFENSE_POSITION_ID_2_BUTTON_PORT		= 2;

#endif /* SRC_PORTS_H_ */
