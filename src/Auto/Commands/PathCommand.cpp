#include <Auto/Commands/PathCommand.h>
#include "CANTalon.h"

PathCommand::PathCommand(RobotModel *robot, Path path) {
	robot_ = robot;
	path_ = path;
	lengthOfLeftMotionProfile_ = 0;
	lengthOfRightMotionProfile_ = 0;

	leftMotionProfileExecutor_ = NULL;
	rightMotionProfileExecutor_ = NULL;
	isDone_ = false;
}

void PathCommand::Init() {
	MotionProfile *motionProfile;
	switch(path_) {
		case(kLiftOne) :
			motionProfile = new LiftOne_MotionProfile();
			break;
		case(kLiftTwo) :
			motionProfile = new LiftTwo_MotionProfile();
			break;
		case(kLiftThree) :
			motionProfile = new LiftThree_MotionProfile();
			break;
		default :
			break;
	}

	lengthOfLeftMotionProfile_ = motionProfile->GetLengthOfLeftMotionProfile();
	lengthOfRightMotionProfile_ = motionProfile->GetLengthOfRightMotionProfile();

	leftMotionProfileExecutor_ = new MotionProfileExecutor(*robot_->leftMaster_, motionProfile->GetLeftMotionProfile(), lengthOfLeftMotionProfile_);
	rightMotionProfileExecutor_ = new MotionProfileExecutor(*robot_->rightMaster_, motionProfile->GetRightMotionProfile(), lengthOfRightMotionProfile_);
}

void PathCommand::Update(double currTimeSec, double deltaTimeSec) {

	leftMotionProfileExecutor_->control();
	rightMotionProfileExecutor_->control();

	robot_->SetMotionProfile();

	CANTalon::SetValueMotionProfile leftSetOutput = leftMotionProfileExecutor_->getSetValue();
	CANTalon::SetValueMotionProfile rightSetOutput = rightMotionProfileExecutor_->getSetValue();

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftSetOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightSetOutput);

	SmartDashboard::PutNumber("Left encoder", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
	SmartDashboard::PutNumber("Right encoder", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));
//	SmartDashboard::PutNumber("Left error", _leftMaster.GetClosedLoopError());
//	SmartDashboard::PutNumber("Right error", _rightMaster.GetClosedLoopError());
//	SmartDashboard::PutNumber("Left speed", _leftMaster.GetSpeed());
//	SmartDashboard::PutNumber("Right speed", _rightMaster.GetSpeed());

	if (!leftMotionProfileExecutor_->hasStarted && !rightMotionProfileExecutor_->hasStarted) {
		leftMotionProfileExecutor_->start();
		rightMotionProfileExecutor_->start();
	}
}

bool PathCommand::IsDone() {
	return isDone_;		// TODO
}

PathCommand::~PathCommand() {
}
