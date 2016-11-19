#ifndef SRC_AUTONOMOUS_MODES_AUTONOMOUSMODE_H_
#define SRC_AUTONOMOUS_MODES_AUTONOMOUSMODE_H_

#include "Autonomous/Commands/AutonomousCommand.h"

class AutonomousMode {
public:
	AutonomousMode();
	virtual ~AutonomousMode() {};

//	virtual void CreateQueue();
//	virtual void AddToQueue(AutonomousCommand *myNewAutoCommand, SimpleAutoCommand *myLastAutoCommand);

	virtual void Update(double currTimeSec, double lastTimeSec) = 0;
	virtual bool IsDone() = 0;
};

#endif /* SRC_AUTONOMOUS_MODES_AUTONOMOUSMODE_H_ */
