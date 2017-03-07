#include <Auto/Commands/AlignWithHighGoalCommand.h>

using namespace std;

AlignWithHighGoalCommand::AlignWithHighGoalCommand(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource) {
	printf("in beginning of alignwithhighgoalcommand\n");

	angleContext_ = new zmq::context_t(1);
	distanceContext_ = new zmq::context_t(1);

	angleSubscriber_ = new zmq::socket_t(*angleContext_, ZMQ_SUB);
	angleSubscriber_->connect("tcp://10.18.68.15:5806");	// MAKE SURE RIGHT IP

	distanceSubscriber_ = new zmq::socket_t(*distanceContext_, ZMQ_SUB);
	distanceSubscriber_->connect("tcp://10.18.68.15:5806");	// MAKE SURE RIGHT IP
	angleSubscriber_->setsockopt( ZMQ_SUBSCRIBE, "ANGLE", 1);
	distanceSubscriber_->setsockopt( ZMQ_SUBSCRIBE, "DISTANCE", 1);

	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	pivotCommand_ = NULL;
	driveStraightCommand_ = NULL;

	isDone_ = false;

	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;

	numTimesInkPivotToAngleInit = 0;

	currState_ = kDriveStraightInit;
	nextState_ = kDriveStraightInit;

	numTimesInkDriveStraightInit = 0;
	printf("in alignWithHighGoalCommand constructor\n");
}

void AlignWithHighGoalCommand::RefreshIni() {
	//pivotCommand_->RefreshIni();
}

void AlignWithHighGoalCommand::Init() {
	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;
	isDone_ = false;

//	currState_ = kPivotToAngleInit;
//	nextState_ = kPivotToAngleInit;
	currState_ = kDriveStraightInit;
	nextState_ = kDriveStraightInit;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	numTimesInkPivotToAngleInit = 0;
	numTimesInkDriveStraightInit = 0;

	printf("in alignwithhighgoalcommand init\n");
}

void AlignWithHighGoalCommand::Update(double currTimeSec, double deltaTimeSec) {
	string angleAddress = s_recv (*angleSubscriber_);
	string angleContents = s_recv (*angleSubscriber_);

	string distanceAddress = s_recv (*distanceSubscriber_);
	string distanceContents = s_recv (*distanceSubscriber_);

	switch (currState_) {
		case (kPivotToAngleInit) :

//			printf("In kPivotToAngleInit\n");
//			// Get angle from Jetson
//			if (angleAddress == "ANGLE") {
//				desiredPivotDeltaAngle_ = stod(angleContents);
//				printf("ANGLE: %f\n", desiredPivotDeltaAngle_);
//			} else {
//				printf("angle address: %s\n", angleAddress.c_str());
//			}
//
//			if (numTimesInkPivotToAngleInit < 3) {
//				numTimesInkPivotToAngleInit++;
//				nextState_ = kPivotToAngleInit;
//			} else if (fabs(desiredPivotDeltaAngle_) > 3.0) {		// 2 degree threshold
//				// CHECK -DESIREDPIVOTDELTAANGLE_
//				// negative because jetson returns angle wrong way
//				printf("ANGLE FOR PIVOT COMMAND: %f\n", -desiredPivotDeltaAngle_);
//				pivotCommand_ = new PivotCommand(robot_, -desiredPivotDeltaAngle_, false, navXSource_);
//				pivotCommand_->Init();
//				nextState_ = kPivotToAngleUpdate;
//			} else {
//				nextState_ = kDriveStraightInit;
//			}
			break;

		case (kPivotToAngleUpdate) :
//			printf("in kPivotToAngleUpdate\n");
//			if (!pivotCommand_->IsDone()) {
//				pivotCommand_->Update(currTimeSec, deltaTimeSec);
//				nextState_ = kPivotToAngleUpdate;
//			} else {
//				nextState_ = kDriveStraightInit;
//			}
			break;

		case (kDriveStraightInit) :
//			driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_,
//												robot_, 1.0);	// converting to feet
//			driveStraightCommand_->Init();
//			nextState_ = kDriveStraightUpdate;
//
//			printf("in kDriveStraightInit\n");

			// Get distance from Jetson
			if (distanceAddress == "DISTANCE") {
				desiredDistance_ = stod(distanceContents);		// IN INCHES
				printf("DISTANCE: %f\n", desiredDistance_);
			}

			if (numTimesInkDriveStraightInit < 3) {
				numTimesInkDriveStraightInit++;
				nextState_ = kDriveStraightInit;
			} else if (fabs(desiredDistance_) > 2.0/12.0) {	// 2 in threshold
				printf("DISTANCE FOR COMMAND: %f\n", desiredDistance_);
				// Jetson returns in inches, so /12.0
				// Subtract 10 inches bc of length of peg
				driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_,
						robot_, (desiredDistance_ - 40.0)/12.0);	// converting to feet
				driveStraightCommand_->Init();
				nextState_ = kDriveStraightUpdate;
			} else {
				isDone_ = true;
			}
			break;

		case (kDriveStraightUpdate) :
			printf("in kDriveStraightUpdate\n");
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

bool AlignWithHighGoalCommand::IsDone() {
	return isDone_;
}

AlignWithHighGoalCommand::~AlignWithHighGoalCommand() {

}
