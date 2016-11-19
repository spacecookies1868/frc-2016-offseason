#include <PIDControlLoop.h>

PIDControlLoop::PIDControlLoop() {
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
	Init(0.0, 0.0);
}

PIDControlLoop::PIDControlLoop(double p, double i, double d) {
	pFac = p;
	iFac = i;
	dFac = d;
	maxAbsOutput = 0.0;
	maxAbsError = 0.0;
	maxAbsDiffError = 0.0;
	desiredAccuracy = 0.0;
	maxAbsITerm = 0.1;
	minAbsError = 0.0;
	timeLimit = 1.5;
	Init(0.0, 0.0);
}

void PIDControlLoop::Init(double myInitialValue, double myDesiredValue) {
	initialValue = myInitialValue;
	desiredValue = myDesiredValue;
}

void PIDControlLoop::Init(double p, double i, double d, double myInitialValue, double myDesiredValue) {
	pFac = p;
	iFac = i;
	dFac = d;
	initialValue = myInitialValue;
	desiredValue = myDesiredValue;
}

double PIDControlLoop::Update(double currentValue) {
	double error = desiredValue - currentValue;
	error = Saturate(error, maxAbsError);
	double diffError = 0.0;

	if (oldError != 0.0) {
		diffError = error - oldError;
		diffError = Saturate(diffError, maxAbsDiffError);
	}

	sumError += error;

	if (iFac > 0.0) {
		sumError = Saturate(sumError, (maxAbsITerm / iFac));
	}

	double pTerm = pFac * error;
	double iTerm = iFac * sumError;
	double dTerm = dFac * diffError;
	double output = pTerm + iTerm + dTerm;
	output = Saturate(output, maxAbsOutput);

	if (fabs(output) < minAbsError) {
		output = 0.0;
	}

	oldError = error;
	return output;
}

double PIDControlLoop::Update(double currentSensorValue, double desiredSensorValue) {
	double error = desiredSensorValue - currentSensorValue;
	desiredValue = desiredSensorValue;
	error = Saturate(error, maxAbsError);
	double diffError = 0.0;

	if (oldError != 0.0) {
		diffError = error - oldError;
		diffError = Saturate(diffError, maxAbsDiffError);
	}

	sumError += error;
	if (iFac > 0.0) {
		sumError = Saturate(sumError, (maxAbsITerm / iFac));
	}

	double pTerm = pFac * error;
	double iTerm = iFac * sumError;
	double dTerm = dFac * diffError;
	double output = pTerm + iTerm + dTerm;
	output = Saturate(output, maxAbsOutput);

	if (fabs(output) < minAbsError) {
		output = 0.0;
	}

	oldError = error;
	return output;
}

/*	When the value returned is close enough to what we want for a certain length of time.
	@param currentSensorValue the current value of the sensor
	@param deltaTime the increment of time between each loop of the code
*/

bool PIDControlLoop::ControlLoopDone(double currentSensorValue, double deltaTime) {
	if (fabs(desiredValue - currentSensorValue) <= desiredAccuracy) {
		timeCount += deltaTime;
		return (timeCount >= timeLimit);
	} else {
		timeCount = 0;
		return false;
	}
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

double PIDControlLoop::GetPFac() {
	return pFac;
}

double PIDControlLoop::GetIFac() {
	return iFac;
}

double PIDControlLoop::GetDFac() {
	return dFac;
}

double PIDControlLoop::Saturate(double value, double maxAbsValue) {
	// Limits the value to maxAbsValue
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
