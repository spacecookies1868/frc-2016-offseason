#include <Autonomous/Commands/DriveStraightCommand.h>

DriveStraightCommand::DriveStraightCommand(RobotModel* myRobot, double myDesiredDis) {
	robot = myRobot;

	lastYaw = 0.0;
	currYaw = 0.0;
	deltaYaw = 0.0;
	accumulatedYaw = 0.0;

	initialR = 0.0;
	desiredR = 0.0;

	desiredDis = myDesiredDis;
	initialDis = 0.0;

	isDone = false;

	disPID = NULL;
	rPID = NULL;
}

void DriveStraightCommand::Init() {
	// Get PFac, IFac, DFac from INI file
	double disPFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disPFac", 0.0);
	double disIFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disIFac", 0.0);
	double disDFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disDFac", 0.0);

	double rPFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rPFac", 0.0);
	double rIFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rIFac", 0.0);
	double rDFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rDFac", 0.0);

	initialDis = (robot->GetLeftDriveEncoderValue() + robot->GetRightDriveEncoderValue()) / 2.0;
	initialR = GetAccumulatedYaw();
	desiredR = 0.0;
	disPID = new PIDControlLoop(disPFac, disIFac, disDFac);
	rPID = new PIDControlLoop(rPFac, rIFac, rDFac);
	disPID->Init(initialDis, initialDis + desiredDis);
	rPID->Init(initialR, initialR + desiredR);
}

double DriveStraightCommand::GetAccumulatedYaw() {
	lastYaw = currYaw;
#if USE_NAVX
	currYaw = robot->GetNavXYaw();
#else
	currYaw = 0.0;
#endif
	deltaYaw = currYaw - lastYaw;

	if (deltaYaw < -180) {			// Going clockwise (from 180 to -180)
		accumulatedYaw += (180 - lastYaw) + (180 + currYaw);
	} else if (deltaYaw > 180) {	// Going counterclockwise (from -180 to 180)
		accumulatedYaw -= (180 + lastYaw) + (180 - currYaw);
	} else {
		accumulatedYaw += deltaYaw;
	}
	return accumulatedYaw;
}

void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) {
	double currDis = (robot->GetLeftDriveEncoderValue() + robot->GetRightDriveEncoderValue()) / 2.0;
	bool disPIDDone = disPID->ControlLoopDone(currDis,deltaTimeSec);

	if (disPIDDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		printf("End distance: %f", currDis);
		printf("End yaw: %f", GetAccumulatedYaw());
	} else {
		double disOutput = disPID->Update(currDis);
		double rOutput = rPID->Update(GetAccumulatedYaw());

		SmartDashboard::PutNumber("Current distance", currDis);
		SmartDashboard::PutNumber("Current yaw", GetAccumulatedYaw());
		SmartDashboard::PutNumber("Distance output", disOutput);
		SmartDashboard::PutNumber("Angle output", rOutput);

		double leftOutput = disOutput + rOutput;
		double rightOutput = disOutput - rOutput;

		// To make sure the output does not exceed 1.0
		double maxOutput = fmax(fabs(leftOutput), fabs(rightOutput));
		if (maxOutput > 1.0) {
			leftOutput = leftOutput / maxOutput;
			rightOutput = rightOutput / maxOutput;
		}

		SmartDashboard::PutNumber("Left output", leftOutput);
		SmartDashboard::PutNumber("Right output", rightOutput);

		robot->SetWheelSpeed(RobotModel::kLeftWheels, leftOutput);
		robot->SetWheelSpeed(RobotModel::kRightWheels, rightOutput);
	}
}

bool DriveStraightCommand::IsDone() {
	return isDone;

}
