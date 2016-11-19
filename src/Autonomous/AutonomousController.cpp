#include <Autonomous/AutonomousController.h>

AutonomousController::AutonomousController(){

}

AutonomousController::AutonomousController(AutonomousMode *autoMode){
	autonomousMode = autoMode;
}

void AutonomousController::SetAutonomousMode(AutonomousMode *autoMode) {
	autonomousMode = autoMode;
}

void AutonomousController::Update(double currTimeSec, double deltaTimeSec) {
	autonomousMode->Update(currTimeSec, deltaTimeSec);
}

void AutonomousController::Reset() {

}

bool AutonomousController::IsDone() {
	return autonomousMode->IsDone();
}
