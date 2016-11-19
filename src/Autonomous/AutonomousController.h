#ifndef SRC_AUTONOMOUS_AUTONOMOUSCONTROLLER_H_
#define SRC_AUTONOMOUS_AUTONOMOUSCONTROLLER_H_

#include "Autonomous/Modes/AutonomousMode.h"

class AutonomousController {
public:
	AutonomousController();
	AutonomousController(AutonomousMode *autoMode);
	virtual ~AutonomousController() {};
	void SetAutonomousMode(AutonomousMode *autoMode);
	void StartAutonomous();		// TODO
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();	// TODO
	bool IsDone();

private:
	AutonomousMode *autonomousMode;
};

#endif /* SRC_AUTONOMOUS_AUTONOMOUSCONTROLLER_H_ */
