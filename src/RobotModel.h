#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include "../ext/navx/AHRS.h"
#include "../ext/ini/ini.h"
#include "Ports.h"
#include <math.h>

class RobotModel {
public:
	RobotModel();
	~RobotModel() {}

	void Reset();

	/* ----- Drive-related methods ----- */
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
	double GetWheelSpeed(Wheels w);
	void SetWheelSpeed(Wheels w, double speed);

	bool IsLowGear();
	void ShiftToLowGear();
	void ShiftToHighGear();

	bool GetBrake();
	void SetBrakeOn();
	void SetBrakeOff();

	double GetLeftDriveEncoderValue();
	double GetRightDriveEncoderValue();
	void ResetDriveEncoders();

	double GetNavXYaw();
	double GetNavXRoll();
	double GetNavXPitch();
	void ZeroNavXYaw();
	/* ------------------------------- */

	/* ----- Super-structure related methods ----- */
	// to do
	/* ----------------------------------------- */

	double GetPressureSensorValue();
	void RefreshIni();

	double GetTime();
	void ResetTimer();

	Ini *pini;		// TODO: clean this
private:
	// Drive actuators
	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB;
	Solenoid *gearShiftSolenoid, *brakeSolenoidA, *brakeSolenoidB;

	// Drive variables
	bool isLowGear;

	// Other
	Compressor *compressor;
	PowerDistributionPanel *pdp;
	Timer *timer;

	// Sensors
	Encoder *leftEncoder, *rightEncoder;
	AnalogInput *pressureSensor;
};

#endif /* SRC_ROBOTMODEL_H_ */
