#ifndef SRC_AUTONOMOUS_MODES_REACHMODE_H_
#define SRC_AUTONOMOUS_MODES_REACHMODE_H_

#include "Autonomous/Commands/DriveStraightCommand.h"
#include "Autonomous/Modes/AutonomousMode.h"
#include "RobotModel.h"

class ReachMode : public AutonomousMode {
public:
	ReachMode(RobotModel *robot);
	virtual ~ReachMode() {};
	void Update(double currTimeSec, double deltaTimeSec) override;
	bool IsDone() override;
private:
	DriveStraightCommand *driveStraightCommand;
	RobotModel *robot;
	bool isDone;
};

#endif /* SRC_AUTONOMOUS_MODES_REACHMODE_H_ */
