#include "Logger.h"

std::ofstream Logger::logData;
std::ofstream Logger::logAction;

void Logger::LogState(RobotModel* myRobot, ControlBoard *myHumanControl) {
	if (!logData.is_open()) {
		logData.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_datalog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
		logData << "Time, Left Encoder, Right Encoder, Left Wheel Speed,"
				<< "Right Wheel Speed, Yaw, Roll, Pitch, Low Gear, Left Joy X, Left Joy Y, "
				<< "Right Joy X, Right Joy Y, Reverse, Arcade, "
				<< "Low Gear Desired, Quick Turn Desired, \r\n";
	}

	logData << myRobot->GetTime() << ", " <<
			myRobot->GetLeftDriveEncoderValue() << ", " <<
			myRobot->GetRightDriveEncoderValue() << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kLeftWheels) << ", " <<
			myRobot->GetWheelSpeed(RobotModel::kRightWheels) << ", " <<
			myRobot->GetNavXYaw() << ", " <<
			myRobot->GetNavXRoll() << ", " <<
			myRobot->GetNavXPitch() << ", " <<
			myRobot->IsLowGear() << ", " <<
			myHumanControl->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kX) << ", " <<
			myHumanControl->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY) << ", " <<
			myHumanControl->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX) << ", " <<
			myHumanControl->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kY) << ", " <<
			myHumanControl->GetReverseDriveDesired() << ", " <<
			myHumanControl->GetArcadeDriveDesired() << ", " <<
			myHumanControl->GetGearShiftDesired() << ", " <<
			myHumanControl->GetQuickTurnDesired() << "\r\n";
	logData.flush();
}

/* Format:
 * RobotModel state / ControlBoard state
 *
 * Timer / left motor / right motor / gear shift / PDP voltage / leftjoy x / leftjoy y
 * 			/ rightjoy x / rightjoy y / reverse desired / gearshift desired /
 */

/* Overloaded methods with time stamp */

void Logger::LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, double state) {
	logAction.flush();
	if (!logAction.is_open()) {
			logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << myRobot->GetTime() << ", " << fileName << ", " << line << ", " << stateName
			<< ", " << state << "\r\n";
	logAction.flush();
}

void Logger::LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state) {
	if (!logAction.is_open()) {
			logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << myRobot->GetTime() << ", " << fileName << ", " << line << ", " << stateName
			<< ", " << state << "\r\n";
	logAction.flush();
}

/* Overloaded methods without time stamp */
void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}

void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}

void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}

void Logger::CloseLogs() {
	logData.close();
	logAction.close();
}

std::string Logger::GetTimeStamp(const char* fileName) {
	time_t rawtime = time(0);
	struct tm * timeinfo;						// Get current time
	char buffer [80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);				// Converts time_t to tm as local time
	strftime (buffer, 80, fileName, timeinfo); 	// FileName contains %F_%H_%M

	return buffer;
}
