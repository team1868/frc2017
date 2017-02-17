#include <Auto/Commands/AlignWithPegCommand.h>

using namespace std;

AlignWithPegCommand::AlignWithPegCommand(RobotModel *robot, NavxPIDSource *navxSource, TalonEncoderPIDSource *talonSource) {
	printf("in beginning of alignwithpegcommand\n");
	angleContext_ = new zmq::context_t(1);
	distanceContext_ = new zmq::context_t(1);

	angleSubscriber_ = new zmq::socket_t(*angleContext_, ZMQ_SUB);
	angleSubscriber_->connect("tcp://10.18.68.40:5563");	// MAKE SURE RIGHT IP

	distanceSubscriber_ = new zmq::socket_t(*distanceContext_, ZMQ_SUB);
	distanceSubscriber_->connect("tcp://10.18.68.40:5563");	// MAKE SURE RIGHT IP
	angleSubscriber_->setsockopt( ZMQ_SUBSCRIBE, "ANGLE", 1);
	distanceSubscriber_->setsockopt( ZMQ_SUBSCRIBE, "DISTANCE", 1);

	robot_ = robot;
	navxSource_ = navxSource;
	talonSource_ = talonSource;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	pivotCommand_ = NULL;
	driveStraightCommand_ = NULL;

	isDone_ = false;

	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;

	currState_ = kPivotToAngleInit;
	nextState_ = kPivotToAngleInit;
	printf("in alignWithPegCommand constructor\n");
}

void AlignWithPegCommand::RefreshIni() {
	//pivotCommand_->RefreshIni();
}

void AlignWithPegCommand::Init() {
	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;
	isDone_ = false;

	currState_ = kPivotToAngleInit;
	nextState_ = kPivotToAngleInit;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	printf("in alignwithpegcommand init\n");
}

void AlignWithPegCommand::Update(double currTimeSec, double deltaTimeSec) {
	string angleAddress = s_recv (*angleSubscriber_);
	string angleContents = s_recv (*angleSubscriber_);

	string distanceAddress = s_recv (*distanceSubscriber_);
	string distanceContents = s_recv (*distanceSubscriber_);

	switch (currState_) {
		case (kPivotToAngleInit) :
			printf("In kPivotToAngleInit\n");
			// Get angle from Jetson
			if (angleAddress == "ANGLE") {
				desiredPivotDeltaAngle_ = stod(angleContents);
				printf("ANGLE: %f\n", desiredPivotDeltaAngle_);
			} else {
				printf("angle address: %s\n", angleAddress.c_str());
			}

			if (fabs(desiredPivotDeltaAngle_) > 1.0) {		// 1 inch threshold
				// CHECK -DESIREDPIVOTDELTAANGLE_
				pivotCommand_ = new PivotCommand(robot_, -desiredPivotDeltaAngle_, false, navxSource_);
				pivotCommand_->Init();
				nextState_ = kPivotToAngleUpdate;
			} else {
				nextState_ = kDriveStraightInit;
			}
			break;

		case (kPivotToAngleUpdate) :
			printf("in kPivotToAngleUpdate\n");
			if (!pivotCommand_->IsDone()) {
				pivotCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kPivotToAngleUpdate;
			} else {
				nextState_ = kDriveStraightInit;
			}
			break;

		case (kDriveStraightInit) :
			// Get distance from Jetson
			if (distanceAddress == "DISTANCE") {
				desiredDistance_ = stod(distanceContents);
				printf("DISTANCE: %f\n", desiredDistance_);
			}

			if (fabs(desiredDistance_) > 1.0/12.0) {		// 1 inch threshold
				// Jetson returns in inches, so /12.0
				// Subtract 10 inches bc of length of peg
				driveStraightCommand_ = new DriveStraightCommand(navxSource_, talonSource_, angleOutput_, distanceOutput_,
						robot_, (desiredDistance_ - 10.0)/12.0);
				driveStraightCommand_->Init();
				nextState_ = kDriveStraightUpdate;
			} else {
				isDone_ = true;
			}
			break;

		case (kDriveStraightUpdate) :
			if (!driveStraightCommand_->IsDone()) {
				driveStraightCommand_->Update(0.0, 0.0); 	// add timer later
				nextState_ = kDriveStraightUpdate;
			} else {
				isDone_ = true;
				// no next state
			}
			break;
	}
	currState_ = nextState_;
}

bool AlignWithPegCommand::IsDone() {
	return isDone_;
}

AlignWithPegCommand::~AlignWithPegCommand() {

}
