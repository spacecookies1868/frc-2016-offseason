/*
 * ReachMode.cpp
 *
 *  Created on: Nov 19, 2016
 *      Author: maggiewang
 */

#include <Autonomous/Modes/ReachMode.h>

ReachMode::ReachMode(RobotModel *myRobot) {
	robot = myRobot;
	driveStraightCommand = new DriveStraightCommand(robot, 4.0);
	driveStraightCommand->Init();
	isDone = false;
}

void ReachMode::Update(double currTimeSec, double deltaTimeSec) {
	if (!driveStraightCommand->IsDone()) {
		driveStraightCommand->Update(currTimeSec, deltaTimeSec);
	} else {
		isDone = true;
	}
}

bool ReachMode::IsDone() {
	return isDone;
}
