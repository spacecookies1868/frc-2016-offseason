#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include "../ext/navx/AHRS.h"
#include "../ext/ini/ini.h"
#include "Logger.h"
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

	double GetDriveLeftEncoderValue();
	double GetDriveRightEncodeValue();
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

private:
	// Drive actuators
	Victor *leftDriveMotorA, *leftDriveMotorB, *rightDriveMotorA, *rightDriveMotorB;
	Solenoid *gearShiftSolenoid;

	// Drive variables
	bool isLowGear;

	// Other
	Compressor *compressor;
	PowerDistributionPanel *pdp;
	Ini *pini;
	Timer *timer;

	// Sensors
	Encoder *leftEncoder, *rightEncoder;
	AnalogInput *pressureSensor;
};

#endif /* SRC_ROBOTMODEL_H_ */
