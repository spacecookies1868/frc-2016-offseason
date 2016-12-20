#ifndef SRC_AUTONOMOUS_COMMANDS_DRIVEBYDISTANCE_H_
#define SRC_AUTONOMOUS_COMMANDS_DRIVEBYDISTANCE_H_

#include "PIDControlLoop.h"
#include "RobotModel.h"

class DriveByDistance {
public:
	DriveByDistance(RobotModel *robot, double distance);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	~DriveByDistance() {};
private:
	RobotModel *robot;
	PIDControlLoop *disPID;
	double initialDistance, desiredDistance;
	bool isDone;
};

#endif /* SRC_AUTONOMOUS_COMMANDS_DRIVEBYDISTANCE_H_ */
