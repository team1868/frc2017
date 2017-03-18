#include <Auto/Commands/PathCommand_OLD.h>
#include "CANTalon.h"

PathCommand_OLD::PathCommand_OLD(RobotModel *robot, Path path) {
	robot_ = robot;
	path_ = path;
	lengthOfLeftMotionProfile_ = 0;
	lengthOfRightMotionProfile_ = 0;

	leftMotionProfileExecutor_ = NULL;
	rightMotionProfileExecutor_ = NULL;
	isDone_ = false;

	leftPFac_ = 0.0;
	leftIFac_ = 0.0;
	leftDFac_ = 0.0;
	leftFFac_ = 0.0;

	rightPFac_ = 0.0;
	rightIFac_ = 0.0;
	rightDFac_ = 0.0;
	rightFFac_ = 0.0;
}

void PathCommand_OLD::Init() {
	robot_->leftMaster_->ClearIaccum();
	robot_->rightMaster_->ClearIaccum();

	robot_->leftMaster_->ClearError();
	robot_->rightMaster_->ClearError();

	MotionProfile *motionProfile;
	switch(path_) {
		case(kLeftLift) :
			printf("Left lift\n");
			motionProfile = new LiftOne_MotionProfile();

			leftPFac_ = robot_->pini_->getf("LEFT LIFT MOTION PROFILE PID", "lPFac", 0.0);
			leftIFac_ = robot_->pini_->getf("LEFT LIFT MOTION PROFILE PID", "lIFac", 0.0);
			leftDFac_ = robot_->pini_->getf("LEFT LIFT MOTION PROFILE PID", "lDFac", 0.0);
			leftFFac_ = robot_->pini_->getf("LEFT LIFT MOTION PROFILE PID", "lFFac", 0.0);

			rightPFac_ = robot_->pini_->getf("LEFT LIFT MOTION PROFILE PID", "rPFac", 0.0);
			rightIFac_ = robot_->pini_->getf("LEFT LIFT MOTION PROFILE PID", "rIFac", 0.0);
			rightDFac_ = robot_->pini_->getf("LEFT LIFT MOTION PROFILE PID", "rDFac", 0.0);
			rightFFac_ = robot_->pini_->getf("LEFT LIFT MOTION PROFILE PID", "rFFac", 0.0);
			break;
		case(kMiddleLift) :
			printf("Middle lift\n");
			motionProfile = new LiftTwo_MotionProfile();

			leftPFac_ = robot_->pini_->getf("MIDDLE LIFT MOTION PROFILE PID", "lPFac", 0.1);
			leftIFac_ = robot_->pini_->getf("MIDDLE LIFT MOTION PROFILE PID", "lIFac", 0.0);
			leftDFac_ = robot_->pini_->getf("MIDDLE LIFT MOTION PROFILE PID", "lDFac", 50.0);
			leftFFac_ = robot_->pini_->getf("MIDDLE LIFT MOTION PROFILE PID", "lFFac", 0.92);

			rightPFac_ = robot_->pini_->getf("MIDDLE LIFT MOTION PROFILE PID", "rPFac", 0.1);
			rightIFac_ = robot_->pini_->getf("MIDDLE LIFT MOTION PROFILE PID", "rIFac", 0.0);
			rightDFac_ = robot_->pini_->getf("MIDDLE LIFT MOTION PROFILE PID", "rDFac", 50.0);
			rightFFac_ = robot_->pini_->getf("MIDDLE LIFT MOTION PROFILE PID", "rFFac", 0.90);
			break;
		case(kRightLift) :
			printf("Right lift\n");
			motionProfile = new LiftThree_MotionProfile();

			leftPFac_ = robot_->pini_->getf("RIGHT LIFT MOTION PROFILE PID", "lPFac", 0.1);
			leftIFac_ = robot_->pini_->getf("RIGHT LIFT MOTION PROFILE PID", "lIFac", 0.0);
			leftDFac_ = robot_->pini_->getf("RIGHT LIFT MOTION PROFILE PID", "lDFac", 62.0);
			leftFFac_ = robot_->pini_->getf("RIGHT LIFT MOTION PROFILE PID", "lFFac", 0.92);

			rightPFac_ = robot_->pini_->getf("RIGHT LIFT MOTION PROFILE PID", "rPFac", 0.1);
			rightIFac_ = robot_->pini_->getf("RIGHT LIFT MOTION PROFILE PID", "rIFac", 0.0);
			rightDFac_ = robot_->pini_->getf("RIGHT LIFT MOTION PROFILE PID", "rDFac", 50.0);
			rightFFac_ = robot_->pini_->getf("RIGHT LIFT MOTION PROFILE PID", "rFFac", 0.90);
			break;
		case(kHighGoalAfterLeftLift) :
			printf("High goal after left lift\n");
			motionProfile = new HighGoalAfterLeftLift_MotionProfile();

			leftPFac_ = robot_->pini_->getf("LEFT HIGH GOAL MOTION PROFILE PID", "lPFac");
			leftIFac_ = robot_->pini_->getf("LEFT HIGH GOAL MOTION PROFILE PID", "lIFac");
			leftDFac_ = robot_->pini_->getf("LEFT HIGH GOAL MOTION PROFILE PID", "lDFac");
			leftFFac_ = robot_->pini_->getf("LEFT HIGH GOAL MOTION PROFILE PID", "lFFac");

			rightPFac_ = robot_->pini_->getf("LEFT HIGH GOAL MOTION PROFILE PID", "rPFac");
			rightIFac_ = robot_->pini_->getf("LEFT HIGH GOAL MOTION PROFILE PID", "rIFac");
			rightDFac_ = robot_->pini_->getf("LEFT HIGH GOAL MOTION PROFILE PID", "rDFac");
			rightFFac_ = robot_->pini_->getf("LEFT HIGH GOAL MOTION PROFILE PID", "rFFac");
			break;
		case(kHighGoalAfterRightLift) :
			printf("High goal after right lift\n");
			motionProfile = new HighGoalAfterRightLift_MotionProfile();

			leftPFac_ = robot_->pini_->getf("RIGHT HIGH GOAL MOTION PROFILE PID", "lPFac", 0.1);
			leftIFac_ = robot_->pini_->getf("RIGHT HIGH GOAL MOTION PROFILE PID", "lIFac", 0.0);
			leftDFac_ = robot_->pini_->getf("RIGHT HIGH GOAL MOTION PROFILE PID", "lDFac", 50.0);
			leftFFac_ = robot_->pini_->getf("RIGHT HIGH GOAL MOTION PROFILE PID", "lFFac", 0.92);

			rightPFac_ = robot_->pini_->getf("RIGHT HIGH GOAL MOTION PROFILE PID", "rPFac", 0.1);
			rightIFac_ = robot_->pini_->getf("RIGHT HIGH GOAL MOTION PROFILE PID", "rIFac", 0.0);
			rightDFac_ = robot_->pini_->getf("RIGHT HIGH GOAL MOTION PROFILE PID", "rDFac", 50.0);
			rightFFac_ = robot_->pini_->getf("RIGHT HIGH GOAL MOTION PROFILE PID", "rFFac)", 0.90);
			break;
		default :
			motionProfile = NULL;
			printf("MOTION PROFILE IS NULL\n");
			break;
	}

	lengthOfLeftMotionProfile_ = motionProfile->GetLengthOfLeftMotionProfile();
	lengthOfRightMotionProfile_ = motionProfile->GetLengthOfRightMotionProfile();

	leftMotionProfileExecutor_ = new MotionProfileExecutor(*robot_->leftMaster_, motionProfile->GetLeftMotionProfile(), lengthOfLeftMotionProfile_);
	rightMotionProfileExecutor_ = new MotionProfileExecutor(*robot_->rightMaster_, motionProfile->GetRightMotionProfile(), lengthOfRightMotionProfile_);

	//ClearMotionProfile();
//	robot_->leftMaster_->ClearIaccum();
//	robot_->rightMaster_->ClearIaccum();
//
//	robot_->leftMaster_->ClearError();
//	robot_->rightMaster_->ClearError();
//
//	robot_->leftMaster_->ClearStickyFaults();
//	robot_->rightMaster_->ClearStickyFaults();

//	robot_->leftMaster_->Reset();
//	robot_->rightMaster_->Reset();
	///

	leftMotionProfileExecutor_->hasStarted_ = false;
	rightMotionProfileExecutor_->hasStarted_ = false;

	leftMotionProfileExecutor_->isDone_ = false;		// unnecessary
	rightMotionProfileExecutor_->isDone_ = false;

//	robot_->leftMaster_->SetPID(0.0, 0.0, 0.0, 0.0);
//	robot_->rightMaster_->SetPID(0.0, 0.0, 0.0, 0.0);
//	robot_->leftSlave_->SetPID(0.0, 0.0, 0.0, 0.0);
//	robot_->rightSlave_->SetPID(0.0, 0.0, 0.0, 0.0);

	// TODO set slot
//	robot_->leftMaster_->SetPID(0.8, 0.0, 0.0, 0.0);
//	robot_->rightMaster_->SetPID(0.8, 0.0, 0.0, 0.0);
//	robot_->leftSlave_->SetPID(0.8, 0.0, 0.0, 0.0);
//	robot_->rightSlave_->SetPID(0.8, 0.0, 0.0, 0.0);
//	robot_->leftMaster_->SetPID(1.0, 0.0, 1.0, 0.0);
//	robot_->rightMaster_->SetPID(1.0, 0.0, 1.0, 0.0);
//	robot_->leftSlave_->SetPID(1.0, 0.0, 1.0, 0.0);
//	robot_->rightSlave_->SetPID(1.0, 0.0, 1.0, 0.0);

	robot_->leftMaster_->SetPID(leftPFac_, leftIFac_, leftDFac_, leftFFac_);
	robot_->rightMaster_->SetPID(rightPFac_, rightIFac_, rightDFac_, rightFFac_);
	robot_->leftSlave_->SetPID(leftPFac_, leftIFac_, leftDFac_, leftFFac_);
	robot_->rightSlave_->SetPID(rightPFac_, rightIFac_, rightDFac_, rightFFac_);

/*	BEFORE COMP VALUES
   robot_->leftMaster_->SetPID(0.9, 0.0, 195.0, 0.85);		// was 0.8 // was 50
	robot_->rightMaster_->SetPID(0.9, 0.0, 210.0, 0.85);
	robot_->leftSlave_->SetPID(0.9, 0.0, 195.0, 0.85);
	robot_->rightSlave_->SetPID(0.9, 0.0, 210.0, 0.85);
*/
//	robot_->leftMaster_->SetPID(0.9, 0.0, 0.0, 0.0);		// was 0.8 // was 50
//	robot_->rightMaster_->SetPID(0.9, 0.0, 0.0, 0.0);
//	robot_->leftSlave_->SetPID(0.9, 0.0, 0.0, 0.0);
//	robot_->rightSlave_->SetPID(0.9, 0.0, 0.0, 0.0);

//	robot_->leftMaster_->SetPID(0.2, 0.0, 10.0, 0.0);		// was 0.8 // was 50
//	robot_->rightMaster_->SetPID(0.2, 0.0, 10.0, 0.0);
//	robot_->leftSlave_->SetPID(0.2, 0.0, 10.0, 0.0);
//	robot_->rightSlave_->SetPID(0.2, 0.0, 10.0, 0.0);

	//	robot_->leftMaster_->SetPID(0.6, 0.0, 0.3, 1.25);
//	robot_->rightMaster_->SetPID(0.6, 0.0, 0.3, 1.5);
//	robot_->leftSlave_->SetPID(0.6, 0.0, 0.3, 1.25);
//	robot_->rightSlave_->SetPID(0.6, 0.0, 0.3, 1.5);
//	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.7, 0.0, 0.2, 1.40329);
//	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.7, 0.0, 0.2, 1.31154);
//	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.6, 0.0, 0.3, 1.25);
//	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.6, 0.0, 0.3, 1.5);

//	leftMotionProfileExecutor_->start();
//	rightMotionProfileExecutor_->start();
//	SmartDashboard::PutBoolean("Left MP Started", leftMotionProfileExecutor_->hasStarted_);;
//	printf("motion profiling started\n");
//	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.0, 0.0, 0.0, 0.0);
//	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.0, 0.0, 0.0, 0.0);
//

//	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.7, 0.02, 0.3, 1.38329);
//	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.7, 0.02, 0.3, 1.30254);
}

void PathCommand_OLD::Update(double currTimeSec, double deltaTimeSec) {
	robot_->SetMotionProfileMode();
	leftMotionProfileExecutor_->control();
	rightMotionProfileExecutor_->control();

	SmartDashboard::PutNumber("Left master state", robot_->leftMaster_->GetControlMode());
	SmartDashboard::PutNumber("Right master state", robot_->rightMaster_->GetControlMode());

	CANTalon::SetValueMotionProfile leftSetOutput = leftMotionProfileExecutor_->getSetValue();
	CANTalon::SetValueMotionProfile rightSetOutput = rightMotionProfileExecutor_->getSetValue();

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftSetOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightSetOutput);

//	SmartDashboard::PutNumber("Left encoder", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
//	SmartDashboard::PutNumber("Right encoder", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));

	SmartDashboard::PutNumber("Left Error", robot_->leftMaster_->GetClosedLoopError());
	SmartDashboard::PutNumber("Right Error", robot_->rightMaster_->GetClosedLoopError());
	SmartDashboard::PutNumber("Left Velocity", robot_->leftMaster_->GetSpeed());
	SmartDashboard::PutNumber("Right Velocity", robot_->rightMaster_->GetSpeed());

//	double leftSpeed = robot_->leftMaster_->GetSpeed();
//	double rightSpeed = robot_->rightMaster_->GetSpeed();
//	SmartDashboard::PutNumber("Left speed", leftSpeed);
//	SmartDashboard::PutNumber("Right speed", rightSpeed);

	if (!leftMotionProfileExecutor_->hasStarted_ && !rightMotionProfileExecutor_->hasStarted_) {
		leftMotionProfileExecutor_->start();
		rightMotionProfileExecutor_->start();
		SmartDashboard::PutBoolean("Left MP Started", leftMotionProfileExecutor_->hasStarted_);
		printf("motion profiling started\n");
	}
}

bool PathCommand_OLD::IsDone() {
	if (leftMotionProfileExecutor_->isDone_ && rightMotionProfileExecutor_->isDone_) { // TODO use IsDone() function
//		robot_->SetPercentVBusDrive();
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
		leftMotionProfileExecutor_->reset();
		rightMotionProfileExecutor_->reset();

		robot_->leftMaster_->ClearIaccum();
		robot_->rightMaster_->ClearIaccum();

		robot_->leftMaster_->ClearError();
		robot_->rightMaster_->ClearError();

		robot_->leftMaster_->ClearMotionProfileTrajectories();
		robot_->rightMaster_->ClearMotionProfileTrajectories();

		robot_->leftMaster_->ClearStickyFaults();
		robot_->rightMaster_->ClearStickyFaults();

		//		ClearMotionProfile();
		robot_->ClearMotionProfileTrajectories();
		printf("PATH COMMAND IS DONE\n");
		return true;
	} else {
		return false;
	}
}

void PathCommand_OLD::ClearMotionProfile() {
//	robot_->leftMaster_->SetPosition(0);
//	robot_->rightMaster_->SetPosition(0);
//
//	robot_->SetPercentVBusDrive();
//	robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
//	leftMotionProfileExecutor_->reset();
//	rightMotionProfileExecutor_->reset();

//	robot_->leftMaster_->ClearIaccum();
//	robot_->rightMaster_->ClearIaccum();
//
//	robot_->leftMaster_->ClearError();
//	robot_->rightMaster_->ClearError();

//	robot_->leftMaster_->ClearStickyFaults();
//	robot_->rightMaster_->ClearStickyFaults();

//	robot_->leftMaster_->Reset();
//	robot_->rightMaster_->Reset();
}

PathCommand_OLD::~PathCommand_OLD() {

}
