#include <RobotModel.h>

RobotModel::RobotModel() {
	// Drive actuators
	leftDriveMotorA = new Victor(LEFT_DRIVE_MOTOR_A_PWM_PORT);
	leftDriveMotorB = new Victor(LEFT_DRIVE_MOTOR_B_PWM_PORT);
	rightDriveMotorA = new Victor(RIGHT_DRIVE_MOTOR_A_PWM_PORT);
	rightDriveMotorB = new Victor(RIGHT_DRIVE_MOTOR_B_PWM_PORT);
	leftDriveMotorA->SetSafetyEnabled(false);
	leftDriveMotorB->SetSafetyEnabled(false);
	rightDriveMotorA->SetSafetyEnabled(false);
	rightDriveMotorB->SetSafetyEnabled(false);

	gearShiftSolenoid = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, GEAR_SHIFT_SOLENOID_PORT);
	brakeSolenoidA = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, BRAKE_SOLENOID_A_PORT);
	brakeSolenoidB = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, BRAKE_SOLENOID_B_PORT);

	// Drive sensors
	leftEncoder = new Encoder(LEFT_ENCODER_A_PWM_PORT, LEFT_ENCODER_B_PWM_PORT, true);
	rightEncoder = new Encoder(RIGHT_ENCODER_A_PWM_PORT, RIGHT_ENCODER_B_PWM_PORT, true);
	leftEncoder->SetDistancePerPulse(((7.4/12.0) * M_PI) / 256.0);		// 7.4 inch wheels, 256 tics per rotation
	rightEncoder->SetDistancePerPulse(((7.4/12.0) * M_PI) / 256.0);

	// Drive variables
	isLowGear = false;

	// Other
	pressureSensor = new AnalogInput(PRESSURE_SENSOR_PORT);
	pressureSensor->SetAverageBits(2);
	compressor = new Compressor(PNEUMATICS_CONTROL_MODULE_ID);
	pdp = new PowerDistributionPanel();
	pini = new Ini("/home/lvuser/robot.ini");
	timer = new Timer();
	timer->Start();
}

void RobotModel::Reset() {
	SetWheelSpeed(RobotModel::kAllWheels, 0.0);
	ResetDriveEncoders();
	RefreshIni();
	ResetTimer();
	ZeroNavXYaw();
}

double RobotModel::GetWheelSpeed(Wheels w) {
	switch (w) {
	case (kLeftWheels):
		return leftDriveMotorA->Get();
		break;
	case (kRightWheels):
		return rightDriveMotorA->Get();
		break;
	default:
		return 0.0;
		break;
	}
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	switch (w) {
	case (kLeftWheels) :
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		break;
	case (kRightWheels) :
		rightDriveMotorA->Set(-speed); // negative value since wheels are inverted on robot
		rightDriveMotorB->Set(-speed);
		break;
	case (kAllWheels) :
		leftDriveMotorA->Set(speed);
		leftDriveMotorB->Set(speed);
		rightDriveMotorA->Set(-speed);
		rightDriveMotorB->Set(-speed);
		break;
	}
}

bool RobotModel::IsLowGear() {
	return isLowGear;
}

void RobotModel::ShiftToLowGear() {
	gearShiftSolenoid->Set(false);
	isLowGear = true;
}

void RobotModel::ShiftToHighGear() {
	gearShiftSolenoid->Set(true);
	isLowGear = false;
}

bool RobotModel::GetBrake() {
	return brakeSolenoidA->Get();
}

void RobotModel::SetBrakeOn() {
	brakeSolenoidA->Set(true);
	brakeSolenoidB->Set(false);
}

void RobotModel::SetBrakeOff() {
	brakeSolenoidA->Set(false);
	brakeSolenoidB->Set(true);
}

double RobotModel::GetLeftDriveEncoderValue() {
	return leftEncoder->GetDistance();
}

double RobotModel::GetRightDriveEncoderValue() {
	return rightEncoder->GetDistance();
}

void RobotModel::ResetDriveEncoders() {
	leftEncoder->Reset();
	rightEncoder->Reset();
}

// TODO: put NavX in
double RobotModel::GetNavXYaw() {
	return 0;
}

double RobotModel::GetNavXRoll() {
	return 0;
}

double RobotModel::GetNavXPitch() {
	return 0;
}

void RobotModel::ZeroNavXYaw() {

}

double RobotModel::GetPressureSensorValue() {
	// TODO
	return 0;
}

void RobotModel::RefreshIni() {
	delete pini;
	pini = new Ini("/home/lvuser/robot.ini");
}

double RobotModel::GetTime() {
	return timer->Get();
}

void RobotModel::ResetTimer() {
	timer->Reset();
}
