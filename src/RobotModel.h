#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include "../ext/navx/AHRS.h"
#include "Logger/Ini.h"
#include "Logger/Logger.h"
#include "Ports.h"

class RobotModel {
public:
	RobotModel();
	~RobotModel();

	/* -----Drive-related methods----- */
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
	void Reset();
	void SetWheelSpeed(Wheels w, double speed);
	double GetWheelSpeed(Wheels w);

	bool IsLowGear();
	bool ShiftToLowGear();
	bool ShiftToHighGear();

	double GetDriveLeftEncoderValue();
	double GetDriveRightEncodeValue();
	void ResetDriveEncoders();

	double GetNavXYaw();
	double GetNavXRoll();
	double GetNavXPitch();
	void ZeroNavXYaw();
	/* ------------------------------- */

	/* -----Super-structure related methods----- */
	// to do
	/* ----------------------------------------- */

	double GetPressureSensorValue();
	void ResetIni();

	double GetTime();
	void ResetTimer();

private:
	// Drive actuators
	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB;
	Solenoid *gearShiftSolenoid;

	// Drive variables
	bool isLowGear;

	/*
	// Superstructure
	Victor *intakeMotor, *outtakeMotorA, *outtakeMotorB; // to do superstructure
	Solenoid *intakeArmSolenoidA, *intakeArmSolenoidB;
	Solenoid *defenseManipSolenoidA, *defenseManipSolenoidB;
	Solenoid *brakeSolenoidA, *brakeSolenoidB;
	*/

	// Other
	Compressor *compressor;
	PowerDistributionPanel *pdp;
	Ini *pini;
	Timer *timer;

	// Sensors
	Encoder *leftEncoder, *rightEncoder;
	//Encoder *outtakeEncoder1, *outtakeEncoder2;
	AnalogInput *pressureSensor;
	DigitalInput *intakeSwitch;
};

#endif /* SRC_ROBOTMODEL_H_ */
