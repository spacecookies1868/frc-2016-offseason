#include <Controllers/DriveController.h>

DriveController::DriveController(RobotModel* myRobot, ControlBoard* myControlBoard) {
	robot = myRobot;
	controlBoard = myControlBoard;
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	robot->ShiftToLowGear();	// always in low gear

	if (controlBoard->GetBrakeDesired()) {
		robot->SetBrakeOn();
	} else {
		robot->SetBrakeOff();
	}

	double rightJoyX = controlBoard->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX);
	double rightJoyY = -controlBoard->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kY);
	double leftJoyY = controlBoard->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY);

	if(controlBoard->GetQuickTurnDesired()) {
		QuickTurn(rightJoyX);
	} else if(controlBoard->GetArcadeDriveDesired()) {
		ArcadeDrive(rightJoyX, leftJoyY);
	} else {
		TankDrive(leftJoyY, rightJoyY);
	}

}

void DriveController::RefreshIni() {
	// TODO
}

void DriveController::QuickTurn(double myRight) {
	robot->SetWheelSpeed(RobotModel::kLeftWheels, myRight);
	robot->SetWheelSpeed(RobotModel::kRightWheels, -myRight);
}

void DriveController::ArcadeDrive(double myX, double myY) {
	double moveValue = myY * DriveDirection();
	double rotateValue = myX;	// TODO: implement drivestraightpid

	if (fabs(moveValue) < 0.1) {
		rotateValue = 0.0;
	}

	double leftMotorOutput = moveValue;
	double rightMotorOutput = moveValue;

	// Make sure -1.0 <= motorOutputs <= 1.0
	if (leftMotorOutput > 1.0) {
		rightMotorOutput = rightMotorOutput / leftMotorOutput;
		leftMotorOutput = 1.0;
	} else if (leftMotorOutput < -1.0) {
		rightMotorOutput = -rightMotorOutput / leftMotorOutput;
		leftMotorOutput = -1.0;
	} else if (rightMotorOutput > 1.0){
		leftMotorOutput = leftMotorOutput / rightMotorOutput;
		rightMotorOutput = 1.0;
	} else if (rightMotorOutput < -1.0) {
		leftMotorOutput = -leftMotorOutput/rightMotorOutput;
		rightMotorOutput = -1.0;
	}

	//Make the joysticks less sensitive
	leftMotorOutput = sin(leftMotorOutput * M_PI / 2);
	rightMotorOutput = sin(rightMotorOutput * M_PI / 2);

	if (fabs(moveValue) < 0.05) {
		leftMotorOutput = 0.0;
		rightMotorOutput = 0.0;
	}

	robot->SetWheelSpeed(RobotModel::kLeftWheels, leftMotorOutput);
	robot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);
}

void DriveController::TankDrive(double myLeft, double myRight) {
	double leftMotorOutput = 0.0;
	double rightMotorOutput = 0.0;

	if (controlBoard->GetReverseDriveDesired()) {
		leftMotorOutput = myRight;
		rightMotorOutput = myLeft;
	} else {
		leftMotorOutput = myLeft;
		rightMotorOutput = myRight;
	}

	leftMotorOutput = sin(leftMotorOutput * M_PI / 2) * DriveDirection();
	rightMotorOutput = sin(rightMotorOutput * M_PI / 2) * DriveDirection();

	robot->SetWheelSpeed(RobotModel::kLeftWheels, leftMotorOutput);
	robot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);
}

int DriveController::DriveDirection() {
	return (controlBoard->GetReverseDriveDesired()) ? -1 : 1;
}

void DriveController::Reset() {

}
