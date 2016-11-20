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
	// Get PID values from INI file
	PIDConfig *disPIDConfig = new PIDConfig();
	// TODO: make this more lightweight somehow
	disPIDConfig->pFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disPFac", 0.6);		// not reading from ini :(. Will fix soon.
	disPIDConfig->iFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disIFac", 0.015);
	disPIDConfig->dFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disDFac", 2.5);
	disPIDConfig->desiredAccuracy = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disDesiredAccuracy", 0.3);
	disPIDConfig->maxAbsOutput = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disMaxAbsOutput", 0.9);
	disPIDConfig->maxAbsError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disMaxAbsError", 4.0);
	disPIDConfig->maxAbsDiffError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disMaxAbsDiffError", 3.0);
	disPIDConfig->maxAbsITerm = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disMaxAbsITerm", 0.4);
	disPIDConfig->timeLimit = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disTimeLimit", 1.0);

	PIDConfig *rPIDConfig = new PIDConfig();
	rPIDConfig->pFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rPFac", 0.02);
	rPIDConfig->iFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rIFac", 0.0);
	rPIDConfig->dFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rDFac", 0.1);
	rPIDConfig->desiredAccuracy = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rDesiredAccuracy", 0.0);
	rPIDConfig->maxAbsOutput = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsOutput", 0.4);
	rPIDConfig->maxAbsError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsError", 0.0);
	rPIDConfig->maxAbsDiffError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsDiffError", 0.0);
	rPIDConfig->maxAbsITerm = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsITerm", 0.0);
	rPIDConfig->timeLimit = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rTimeLimit", 0.0);

	initialDis = (robot->GetLeftDriveEncoderValue() + robot->GetRightDriveEncoderValue()) / 2.0;
	initialR = GetAccumulatedYaw();
	desiredR = 0.0;
	disPID = new PIDControlLoop(disPIDConfig);
	rPID = new PIDControlLoop(rPIDConfig);
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
	bool disPIDDone = disPID->ControlLoopDone(currDis);

	if (disPIDDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		printf("End distance: %f\n", currDis);
		printf("End yaw: %f\n", GetAccumulatedYaw());
	} else {
		double disOutput = disPID->Update(currDis);
		double rOutput = rPID->Update(GetAccumulatedYaw());
		disPID->PrintPIDValues("disPID");
		rPID->PrintPIDValues("rPID");

		SmartDashboard::PutNumber("Current distance", currDis);
		SmartDashboard::PutNumber("Current yaw", GetAccumulatedYaw());
		SmartDashboard::PutNumber("Distance output", disOutput);
		SmartDashboard::PutNumber("Angle output", rOutput);
		SmartDashboard::PutNumber("Distance error", disPID->GetError());
		SmartDashboard::PutNumber("Angle error", rPID->GetError());

		printf("Distance error: %f\n", disPID->GetError());
		printf("Angle error: %f\n", rPID->GetError());

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
