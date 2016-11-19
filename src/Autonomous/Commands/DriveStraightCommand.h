#ifndef SRC_AUTONOMOUS_COMMANDS_DRIVESTRAIGHTCOMMAND_H_
#define SRC_AUTONOMOUS_COMMANDS_DRIVESTRAIGHTCOMMAND_H_

#include "WPILib.h"
#include "AutonomousCommand.h"
#include "Controllers/DriveController.h"
#include "PIDControlLoop.h"
#include "RobotModel.h"

class DriveStraightCommand {
public:
	DriveStraightCommand(RobotModel *myRobot, double myDesiredDis);
	~DriveStraightCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	double GetAccumulatedYaw();

	RobotModel *robot;
	PIDControlLoop *disPID, *rPID;

	double lastYaw, currYaw, deltaYaw, accumulatedYaw;
	double desiredDis, initialDis;
	double desiredR, initialR;

	bool isDone;
};

#endif /* SRC_AUTONOMOUS_COMMANDS_DRIVESTRAIGHTCOMMAND_H_ */
