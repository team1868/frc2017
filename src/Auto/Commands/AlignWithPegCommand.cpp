#include <Auto/Commands/AlignWithPegCommand.h>

using namespace std;

AlignWithPegCommand::AlignWithPegCommand(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, bool isDriveStraightDesired) {
	printf("in beginning of alignwithpegcommand\n");

//	visionLog_.open("/tmp/vision_log.txt");

//	angleContext_ = new zmq::context_t(1);
//	distanceContext_ = new zmq::context_t(1);
//	context_ = new zmq::context_t(1);

//	angleSubscriber_ = new zmq::socket_t(*angleContext_, ZMQ_SUB);
//	angleSubscriber_->connect("tcp://10.18.68.15:5563");	// MAKE SURE RIGHT IP
//
//	distanceSubscriber_ = new zmq::socket_t(*distanceContext_, ZMQ_SUB);
//	distanceSubscriber_->connect("tcp://10.18.68.15:5563");	// MAKE SURE RIGHT IP
//	angleSubscriber_->setsockopt( ZMQ_SUBSCRIBE, "ANGLE", 1);
//	distanceSubscriber_->setsockopt( ZMQ_SUBSCRIBE, "DISTANCE", 1);

	context_ = NULL;
	subscriber_ = NULL;
//	subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
//	subscriber_->connect("tcp://10.18.68.15:5563");	// MAKE SURE RIGHT IP
//	subscriber_->setsockopt(ZMQ_SUBSCRIBE, "MESSAGE", 0);

	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;
//	isDriveStraightDesired_ = isDriveStraightDesired;
	isDriveStraightDesired_ = false;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	pivotCommand_ = NULL;
	driveStraightCommand_ = NULL;

	isDone_ = false;

	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;

	numTimesInkPivotToAngleInit = 0;

	currState_ = kPivotToAngleInit;
	nextState_ = kPivotToAngleInit;

	numTimesInkDriveStraightInit = 0;
	timeStartForVision_ = 0.0;
	printf("in alignWithPegCommand constructor\n");
}

void AlignWithPegCommand::RefreshIni() {
	//pivotCommand_->RefreshIni();
}

void AlignWithPegCommand::Init() {
	printf("in alignwithpegcommand init\n");
	Profiler profiler(robot_, "Align With Peg Init");

	context_ = new zmq::context_t(1);

	try {
		subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
		subscriber_->connect("tcp://10.18.68.15:5563");	// MAKE SURE RIGHT IP
		int confl = 1;
		subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
		subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000);
		subscriber_->setsockopt(ZMQ_SUBSCRIBE, "MESSAGE", 0);
	} catch(const zmq::error_t &exc) {
		std::cerr << exc.what();
	}

//	subscriber_->setsockopt(ZMQ_CONFLATE, "MESSAGE", 0);
//	subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0);

	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;
	isDone_ = false;

	currState_ = kPivotToAngleInit;
	nextState_ = kPivotToAngleInit;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	numTimesInkPivotToAngleInit = 0;
	numTimesInkDriveStraightInit = 0;

}

void AlignWithPegCommand::Update(double currTimeSec, double deltaTimeSec) {
//	string angleAddress = s_recv (*angleSubscriber_);
//	string angleContents = s_recv (*angleSubscriber_);
//
//	string distanceAddress = s_recv (*distanceSubscriber_);
//	string distanceContents = s_recv (*distanceSubscriber_);
	//	string address = s_recv(*subscriber_);
	//	string contents = s_recv (*subscriber_);

	//	cout << "address " << address << endl;
	//	cout << "contents " << contents << endl;

	double lastDesiredAngle = desiredPivotDeltaAngle_;
	double lastDesiredDistance = desiredDistance_;

	ReadFromJetson();

	SmartDashboard::PutNumber("Vision pivot delta angle", desiredPivotDeltaAngle_);
	SmartDashboard::PutNumber("Vision desired distance", desiredDistance_);

	switch (currState_) {
		case (kPivotToAngleInit) :
			printf("In kPivotToAngleInit\n");

//			// Get angle from Jetson
//			if (angleAddress == "ANGLE") {
//				desiredPivotDeltaAngle_ = stod(angleContents);
//				printf("ANGLE: %f\n", desiredPivotDeltaAngle_);
//			} else {
//				printf("angle address: %s\n", angleAddress.c_str());
//			}

//			ReadUpdateFromJetson();

			if (fabs(lastDesiredAngle - desiredPivotDeltaAngle_) > 2.0) {
				nextState_ = kPivotToAngleInit;
			} else if (fabs(desiredPivotDeltaAngle_) > 2.0) {
//			if (numTimesInkPivotToAngleInit < 3) {
//				numTimesInkPivotToAngleInit++;
//				nextState_ = kPivotToAngleInit;
//			} else if (fabs(desiredPivotDeltaAngle_) > 3.0) {		// 2 degree threshold
				// CHECK -DESIREDPIVOTDELTAANGLE_
				// negative because jetson returns angle wrong way
				printf("vision done at: %f\n", robot_->GetTime() - timeStartForVision_);

				printf("ANGLE FOR PIVOT COMMAND: %f\n", -desiredPivotDeltaAngle_);
				pivotCommand_ = new PivotCommand(robot_, -desiredPivotDeltaAngle_, false, navXSource_);
				printf("pivotCommand constructed: %f\n", robot_->GetTime() - timeStartForVision_);
				pivotCommand_->Init();
				printf("pivotCommand inited: %f\n", robot_->GetTime() - timeStartForVision_);
				nextState_ = kPivotToAngleUpdate;
			} else {
				printf("vision done at: %f\n", robot_->GetTime() - timeStartForVision_);
				printf("ANGLE THAT WAS GOOD NO PIVOT: %f\n", -desiredPivotDeltaAngle_);
				nextState_ = kDriveStraightInit;
//				isDone_ = true;
			}
			break;

		case (kPivotToAngleUpdate) :
			printf("in kPivotToAngleUpdate\n");
			if (!pivotCommand_->IsDone()) {
				pivotCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kPivotToAngleUpdate;
			} else {
				if (isDriveStraightDesired_) {
					nextState_ = kDriveStraightInit;
				} else {
					isDone_ = true;
				}			}
			break;

		case (kDriveStraightInit) :
			printf("In DriveStraightInit\n");
//			// Get distance from Jetson
//			if (distanceAddress == "DISTANCE") {
//				desiredDistance_ = stod(distanceContents);		// IN INCHES
//				printf("DISTANCE: %f\n", desiredDistance_);
//			} else {
//				printf("distance address: %s\n", distanceAddress.c_str());
//			}
//
//			if (numTimesInkDriveStraightInit < 3) {
//				numTimesInkDriveStraightInit++;
//				nextState_ = kDriveStraightInit;
			if (fabs(desiredDistance_) > 2.0/12.0) {	// 2 in threshold
				printf("DISTANCE FOR COMMAND: %f\n", desiredDistance_);
				// Jetson returns in inches, so /12.0
				// Subtract 10 inches bc of length of peg
				// negative because technically driving backwards
				driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_,
						robot_, -(desiredDistance_ - 13.0)/12.0);	// converting to feet
				driveStraightCommand_->Init();
				nextState_ = kDriveStraightUpdate;
			} else {
				isDone_ = true;
			}
			break;

		case (kDriveStraightUpdate) :
			printf("In DriveStraightUpdate \n");
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
	if (isDone_) {
		subscriber_->close();
	}

	return isDone_;
}

void AlignWithPegCommand::ReadFromJetson() {
//	zmq::message_t message;
//	int tosses = 0;
//
//	while((subscriber_->recv(&message,ZMQ_NOBLOCK)) == 0) {
//		tosses += 1;
//	}
//
//	string contents(static_cast<char*>(message.data()), message.size());
	Profiler profilerFromJetson(robot_, "ReadFromJetson");
	printf("in front of read from jetson\n");
//	if (subscriber_ != NULL) {
	try {
		Profiler profilerInTry(robot_, "ProfilerInTry");
		string contents = s_recv(*subscriber_);

		stringstream ss(contents);
		vector<string> result;

		while(ss.good()) {
			string substr;
			getline( ss, substr, ' ' );
			result.push_back( substr );
		}

		desiredPivotDeltaAngle_ = stod(result.at(0));
		desiredDistance_ = stod(result.at(1));
//	}
	} catch (const std::exception &exc) {
		std::cerr << exc.what();
		desiredPivotDeltaAngle_ = 0.0;
		desiredDistance_ = 0.0;
	}
	printf("in end of read from jetson\n");
//	SmartDashboard::PutNumber("Subscriber Tosses", tosses);
}

AlignWithPegCommand::~AlignWithPegCommand() {

}
