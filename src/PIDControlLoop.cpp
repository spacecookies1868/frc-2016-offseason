#include <PIDControlLoop.h>

PIDConfig::PIDConfig() {
	pFac = 0.0;
	iFac = 0.0;
	dFac = 0.0;
	maxAbsOutput = 0.0;
	maxAbsError = 0.0;
	maxAbsDiffError = 0.0;
	desiredAccuracy = 0.0;
	maxAbsITerm = 0.1;
	minAbsError = 0.0;
	timeLimit = 1.5;
}

PIDControlLoop::PIDControlLoop(PIDConfig *myConfig) {
	pidConfig = myConfig;
	Init(0.0, 0.0);
}

void PIDControlLoop::Init(double myInitialSensorValue, double myDesiredSensorValue) {
	initialSensorValue = myInitialSensorValue;
	desiredSensorValue = myDesiredSensorValue;
	oldError = 0.0;
	sumError = 0.0;
	timeWithinSetpointThreshold = 0.0;
}

void PIDControlLoop::Init(PIDConfig *myConfig, double myInitialSensorValue, double myDesiredSensorValue) {
	pidConfig = myConfig;
	Init(myInitialSensorValue, myDesiredSensorValue);
	timeWithinSetpointThreshold = 0.0;
}

// Returns the actuator value (motor speed, etc.)
double PIDControlLoop::Update(double myCurrentSensorValue) {
	currentSensorValue = myCurrentSensorValue;
	error = desiredSensorValue - currentSensorValue;
	error = Saturate(error, pidConfig->maxAbsError);
	diffError = 0.0;
	if (oldError != 0.0) {
		diffError = error - oldError;
		diffError = Saturate(diffError, pidConfig->maxAbsDiffError);
	}
	sumError += error;
	if (pidConfig->iFac > 0.0) {
		sumError = Saturate(sumError, (pidConfig->maxAbsITerm / pidConfig->iFac));
	}
	pTerm = pidConfig->pFac * error;
	iTerm = pidConfig->iFac * sumError;
	dTerm = pidConfig->dFac * diffError;
	output = pTerm + iTerm + dTerm; //  PID
	output = Saturate(output, pidConfig->maxAbsOutput);
	oldError = error;
	return output;
}

double PIDControlLoop::Update(double currValue, double desiredValue) {
	error = desiredValue - currValue;
	desiredSensorValue = desiredValue;
	error = Saturate(error, pidConfig->maxAbsError);
	diffError = 0.0;
	if (oldError != 0.0) {
		diffError = error - oldError;
		diffError = Saturate(diffError, pidConfig->maxAbsDiffError);
	}
	sumError += error;
	if (pidConfig->iFac > 0.0) {
		sumError = Saturate(sumError,
				(pidConfig->maxAbsITerm / pidConfig->iFac));
	}
	pTerm = pidConfig->pFac * error;
	iTerm = pidConfig->iFac * sumError;
	dTerm = pidConfig->dFac * diffError;
	output = pTerm + iTerm + dTerm; //  PID
	output = Saturate(output, pidConfig->maxAbsOutput);
	if (fabs(output) < pidConfig->minAbsError) {
		output = 0.0;
	}
	oldError = error;
	return output;
}

double PIDControlLoop::Update(PIDConfig* myConfig, double currValue, double desiredValue) {
	pidConfig = myConfig;
	return Update(currValue, desiredValue);
}

// Limits value to maxAbsValue
double PIDControlLoop::Saturate(double value, double maxAbsValue) {
	if (maxAbsValue > 0.0) {
		if (value > 0.0) {
			return fmin(value, maxAbsValue);
		} else {
			return fmax(value, -maxAbsValue);
		}
	} else {
		return value;
	}
}

bool PIDControlLoop::ControlLoopDone(double currentSensorValue) {
	double deltaTime = 0.02;	// The increment of time between each loop of the code
	if (fabs(desiredSensorValue - currentSensorValue) <= pidConfig->desiredAccuracy) {
		timeWithinSetpointThreshold += deltaTime;	// Increment timeWithinSetpointThreshold when the process variable is within the threshold

		if (timeWithinSetpointThreshold >= pidConfig->timeLimit) {	// Return true when the counter has reached the time limit
			return true;
		} else {
			return false;
		}
	} else {
		timeWithinSetpointThreshold = 0;	// If the process variable exits the desiredAccuracy, reset the counter
		return false;
	}
}

PIDConfig* PIDControlLoop::GetPIDConfig() {
	return pidConfig;
}

double PIDControlLoop::GetError() {
	return error;
}

double PIDControlLoop::GetDiffError() {
	return diffError;
}

double PIDControlLoop::GetSumError() {
	return sumError;
}

double PIDControlLoop::GetPTerm() {
	return pTerm;
}

double PIDControlLoop::GetITerm() {
	return iTerm;
}

double PIDControlLoop::GetDTerm() {
	return dTerm;
}

void PIDControlLoop::PrintPIDValues(const std::string& pidName) {
	PrintPIDToConsole(pidName);
	PrintPIDToSmartDashboard(pidName);
}

void PIDControlLoop::PrintPIDToConsole(const std::string& pidName) {
	printf("Current Sensor Value: %f\n", currentSensorValue);
	printf("Error: %f, DiffError: %f, SumErrorL %f\n", error, diffError, sumError);
	printf("PTerm: %f, ITerm: %f, DTerm: %f, Output: %f\n", pTerm, iTerm, dTerm, output);
	printf("Error: %f, DiffError: %f, SumErrorL %f\n", error, diffError, sumError);
	printf("PFac: %f, IFac: %f, DFac: %f\n", pidConfig->pFac, pidConfig->iFac, pidConfig->dFac);
}

void PIDControlLoop::PrintPIDToSmartDashboard(const std::string& pidName) {
	SmartDashboard::PutNumber(pidName + " error", error);
	SmartDashboard::PutNumber(pidName + " sumError", sumError);
	SmartDashboard::PutNumber(pidName + " diffError", diffError);
	SmartDashboard::PutNumber(pidName + " pTerm", pTerm);
	SmartDashboard::PutNumber(pidName + " iTerm", iTerm);
	SmartDashboard::PutNumber(pidName + " dTerm", dTerm);
}
