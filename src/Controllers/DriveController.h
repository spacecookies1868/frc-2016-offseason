#ifndef SRC_CONTROLLERS_DRIVECONTROLLER_H_
#define SRC_CONTROLLERS_DRIVECONTROLLER_H_

#include "WPILib.h"
#include "ControlBoard.h"
#include "Logger.h"
#include "RobotModel.h"
#include <math.h>

class DriveController {
public:
	DriveController(RobotModel* robot, ControlBoard* controlBoard);
	~DriveController() {}
	void Update(double currTimeSec, double deltaTimeSec);
	void RefreshIni();
	void Reset();
	void QuickTurn(double myRight);
	void ArcadeDrive(double myX, double myY);
	void TankDrive(double myLeft, double myRight);
	int DriveDirection();

	enum DriveState {kInitialize, kTeleopDrive};

private:
	RobotModel *robot;
	ControlBoard *controlBoard;
};

#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */
