/*
 * PIDControlLoop.h
 *
 *  Created on: Nov 16, 2016
 *      Author: maggiewang
 */

#ifndef SRC_PIDCONTROLLOOP_H_
#define SRC_PIDCONTROLLOOP_H_

class PIDControlLoop {
public:
	PIDControlLoop();
	~PIDControlLoop() {}

	void Init(double initialValue, double desiredValue);
	void Init(double pFac, double iFac, double dFac,
			  double initialValue, double desiredValue);
//	void Set(double pFac, double iFac, double dFac,
//			 double maxAbsOutput, double maxAbsError, double maxAbsDiffError,
//			 double desiredAccuracy, double maxAbsIFac, double minAbsError,
//			 double timeLimit);
	void Set(double pFac, double iFac, double dFac);
	double Update(double currentValue);
	double Update(double currentValue, double desiredValue);
	bool ControlLoopDone(double currentValue);
	bool ControlLoopDone(double currentValue, double deltaTime);

	double Saturate(double value, double maxAbsValue);

	// Accessors
	double GetError();
	double GetDiffError();
	double GetSumError();
	double GetPFac();
	double GetIFac();
	double GetDFac();

	void PrintPIDValues(std::string pidName);

private:
	// PID variables
	double pFac, iFac, dFac;
	double maxAbsOutput, maxAbsError, maxAbsDiffError;
	double desiredAccuracy, maxAbsIFac, minAbsError;
	double timeLimit;

	//
	double pTerm, iTerm, dTerm;
	double initialValue, desiredValue;
	double oldError, error, sumError, diffError;
	double timeCount;
};

#endif /* SRC_PIDCONTROLLOOP_H_ */
