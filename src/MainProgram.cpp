#include "WPILib.h"
#include "../ext/ini/ini.h"
#include "Autonomous/Modes/ReachMode.h"
#include "Autonomous/AutonomousController.h"
#include "Controllers/DriveController.h"
#include "Controllers/SuperstructureController.h"
#include "Logger.h"
#include "Vision/CameraController.h"
#include "ControlBoard.h"
#include "RobotModel.h"

class MainProgram : public IterativeRobot {
	// Declares pointers
	RobotModel *robot;
	ControlBoard *controlBoard;
	DriveController *driveController;
	SuperstructureController *superstructureController;
	AutonomousController *autonomousController;
	CameraController *cameraController;
	LiveWindow *liveWindow;

	// Timer variables
	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;

public:
	MainProgram() {
		robot = new RobotModel();
		controlBoard = new ControlBoard();
		driveController = new DriveController(robot, controlBoard);
		superstructureController = new SuperstructureController();
		cameraController = new CameraController();
		liveWindow = LiveWindow::GetInstance();

		ResetTimerVariables();
	}

private:
	void RobotInit() {
		robot->Reset();
		RefreshAllIni();
	}

	void AutonomousInit() {
		ResetTimerVariables();
		RefreshAllIni();
		ResetControllers();

		// Starts autonomous
		// autonomousController->StartAutonomous();
		autonomousController = new AutonomousController();
		ReachMode *reachMode = new ReachMode(robot);
		autonomousController->SetAutonomousMode(reachMode);
		Wait(0.5);
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		if (!autonomousController->IsDone()) {	// TODO get rid of the isdone and put it in automode
			autonomousController->Update(currTimeSec, deltaTimeSec);
		}
	}

	void TeleopInit() {
		ResetTimerVariables();
		RefreshAllIni();
		ResetControllers();
	}

	void TeleopPeriodic() {
		UpdateTimerVariables();

		// Reads controls and updates controllers
		controlBoard->ReadControls();
		driveController->Update(currTimeSec, deltaTimeSec);
	}

	void TestPeriodic() {
		UpdateTimerVariables();
		// TODO
	}

	void RefreshAllIni() {
		robot->RefreshIni();
		driveController->RefreshIni();
	}

	void ResetControllers() {
		robot->Reset();
		driveController->Reset();
		autonomousController->Reset();
	}

	void ResetTimerVariables() {
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

	void UpdateTimerVariables() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;
	}
};

START_ROBOT_CLASS(MainProgram)
