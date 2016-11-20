#ifndef SRC_PIDCONTROLLOOP_H_
#define SRC_PIDCONTROLLOOP_H_

#include "WPILib.h"
#include <math.h>
#include <string.h>

struct PIDConfig {
 public:
	PIDConfig();
	double pFac;
	double iFac;
	double dFac;
	double maxAbsOutput;
	double maxAbsError;
	double maxAbsDiffError;
	double desiredAccuracy;
	double maxAbsITerm;
	double minAbsError;
	double timeLimit;
};

class PIDControlLoop {
 public:
	PIDControlLoop(PIDConfig *pidConfig);
	~PIDControlLoop() {}

	void Init(double initValue, double desiredValue);
	void Init(PIDConfig *pidConfig, double initValue, double desiredValue);

	double Update(double currValue); // Returns the actuator value (motor speed, etc.)
	double Update(double currValue, double desiredValue);
	double Update(PIDConfig *pidConfig, double currValue, double desiredValue);

	bool ControlLoopDone(double currValue);

	static double Saturate(double value, double maxAbsValue);

	PIDConfig *GetPIDConfig();
	double GetError();
	double GetDiffError();
	double GetSumError();
	double GetPTerm();
	double GetITerm();
	double GetDTerm();

	void PrintPIDValues(const std::string& pidName);
	void PrintPIDToConsole(const std::string& pidName);
	void PrintPIDToSmartDashboard(const std::string& pidName);

private:
	PIDConfig *pidConfig;
	double initialSensorValue, currentSensorValue, desiredSensorValue;
	double oldError, error, sumError, diffError;
	double pTerm, iTerm, dTerm;
	double output;
	double timeWithinSetpointThreshold;
};

#endif /* SRC_PIDCONTROLLOOP_H_ */
