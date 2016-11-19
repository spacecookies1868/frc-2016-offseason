#ifndef SRC_LOGGER_H_
#define SRC_LOGGER_H_

#include "ControlBoard.h"
#include "RobotModel.h"
#include <fstream>
#include <string>
#include <ctime>

//#define LOG(myRobot, stateName, state) {MATCH_PERIODIC(1, Logger::LogAction(myRobot, __FILE__, __LINE__, stateName, state))}
//#define DUMP(stateName, state) {MATCH_PERIODIC(1, Logger::LogAction(__FILE__, __LINE__, stateName, state))}

class Logger {
public:
	// LogState records the physical state of the robot and human control
	static void LogState(RobotModel* myRobot, ControlBoard *myHumanControl);

	// LogAction records higher-level processes
	static void LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, double state);
	static void LogAction(RobotModel *myRobot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state);

	static std::string GetTimeStamp(const char *fileName);
	static void CloseLogs();
private:
	static std::ofstream logData, logAction;
};

#endif /* SRC_LOGGER_H_ */
