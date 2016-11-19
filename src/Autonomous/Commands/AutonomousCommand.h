#ifndef SRC_AUTONOMOUS_COMMANDS_AUTONOMOUSCOMMAND_H_
#define SRC_AUTONOMOUS_COMMANDS_AUTONOMOUSCOMMAND_H_

#include "WPILib.h"

class AutonomousCommand {
public:
	AutonomousCommand() {}
	virtual ~AutonomousCommand() {}
	virtual void Init() = 0;
	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;
	virtual bool IsDone() = 0;
	virtual AutonomousCommand *GetNextCommand() = 0;
};

class SimpleAutoCommand : public AutonomousCommand {
public:
	SimpleAutoCommand() {
		nextCommand = NULL;
	}
	virtual ~SimpleAutoCommand() {}
	virtual void SetNextCommand(AutonomousCommand *myNextCommand) {
		nextCommand = myNextCommand;
	}
	virtual AutonomousCommand *GetNextCommand() {
		return nextCommand;
	}

private:
	AutonomousCommand *nextCommand;
};

#endif /* SRC_AUTONOMOUS_COMMANDS_AUTONOMOUSCOMMAND_H_ */
