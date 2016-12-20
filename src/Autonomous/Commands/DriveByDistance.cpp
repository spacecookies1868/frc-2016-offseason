#include <Autonomous/Commands/DriveByDistance.h>

DriveByDistance::DriveByDistance(RobotModel *myRobot, double myDesiredDistance) {
	robot = myRobot;
	initialDistance = 0.0;
	desiredDistance = myDesiredDistance;
	isDone = false;
	disPID = NULL;
}

void DriveByDistance::Init() {
	PIDConfig *disPIDConfig = new PIDConfig();
	disPIDConfig->pFac = 0.0;
	disPIDConfig->iFac = 0.0;
	disPIDConfig->dFac = 0.0;
	disPID = new PIDControlLoop(disPIDConfig);
	disPID->Init(disPIDConfig, initialDistance, initialDistance + desiredDistance);
}

void DriveByDistance::Update(double currTimeSec, double deltaTimeSec) {
	double currentDistance = (robot->GetLeftDriveEncoderValue() + robot->GetRightDriveEncoderValue()) / 2;
	isDone = disPID->ControlLoopDone(currentDistance);

	if (!isDone) {
		double disPIDOutput = disPID->Update(currentDistance);
		if (disPIDOutput > 1.0) {
			disPIDOutput = disPIDOutput / 1.0;
		}
		robot->SetWheelSpeed(RobotModel::kAllWheels, disPIDOutput);
	} else {
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
	}
	disPID->PrintPIDValues("disPID");
}

bool DriveByDistance::IsDone() {
	return isDone;
}
