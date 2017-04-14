#include <Auto/Commands/AlignWithPegCommand.h>

using namespace std;

AlignWithPegCommand::AlignWithPegCommand(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, bool isDriveStraightDesired) {
	printf("in beginning of alignwithpegcommand\n");

	context_ = NULL;
	subscriber_ = NULL;

	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;
	isDriveStraightDesired_ = isDriveStraightDesired;

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
	timeStartForAlignWithPegCommand_ = 0.0;
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
		printf("TRY CATCH FAILED IN ALIGNWITHPEGCOMMAND INIT\n");
		std::cerr << exc.what();
	}

	desiredPivotDeltaAngle_ = 0.0;
	desiredDistance_ = 0.0;
	isDone_ = false;

	currState_ = kPivotToAngleInit;
	nextState_ = kPivotToAngleInit;

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	numTimesInkPivotToAngleInit = 0;
	numTimesInkDriveStraightInit = 0;

	timeStartForAlignWithPegCommand_ = robot_->GetTime();
}

void AlignWithPegCommand::Update(double currTimeSec, double deltaTimeSec) {
	double lastDesiredAngle = desiredPivotDeltaAngle_;
	double lastDesiredDistance = desiredDistance_;
	double diffInAngle;

	switch (currState_) {
		case (kPivotToAngleInit) :
			printf("In kPivotToAngleInit\n");

			ReadFromJetson();
			SmartDashboard::PutNumber("Vision pivot delta angle", desiredPivotDeltaAngle_);
			SmartDashboard::PutNumber("Vision desired distance", desiredDistance_);

			diffInAngle = fabs(lastDesiredAngle - desiredPivotDeltaAngle_);
			printf("Difference in Angle: %f\n", diffInAngle);
			if (diffInAngle > 2.0 || (diffInAngle == 0.0 && desiredPivotDeltaAngle_ == 0.0)) {
				printf("diff in angle is > 2\n");
				nextState_ = kPivotToAngleInit;
			} else if (fabs(desiredPivotDeltaAngle_) > 2.0) {
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
			}
			break;

		case (kPivotToAngleUpdate) :
			if (!pivotCommand_->IsDone()) {
				pivotCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kPivotToAngleUpdate;
			} else {
				printf("Pivot To Angle Is Done\n");
				if (isDriveStraightDesired_) {
					nextState_ = kDriveStraightInit;
				} else {
					isDone_ = true;
					printf("AlighWithPeg Done \n");
				}
			}
			break;

		case (kDriveStraightInit) :
			printf("In DriveStraightInit\n");

			ReadFromJetson();

			if (fabs(desiredDistance_) > 2.0/12.0) {	// 2 in threshold
				printf("DISTANCE FOR COMMAND: %f\n", desiredDistance_);
				// Jetson returns in inches, so /12.0
				// Subtract 10 inches bc of length of peg
				// negative because technically driving backwards
				driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_,
						robot_, -(desiredDistance_ - 10.0)/12.0);	// converting to feet
				driveStraightCommand_->Init();
				nextState_ = kDriveStraightUpdate;
			} else {
				isDone_ = true;
				printf("Done with AlignWithPeg \n");
			}
			break;

		case (kDriveStraightUpdate) :
			if (!driveStraightCommand_->IsDone()) {
				driveStraightCommand_->Update(0.0, 0.0); 	// add timer later
				nextState_ = kDriveStraightUpdate;
			} else {
				isDone_ = true;
				// no next state
				printf("Done with AlignWithPeg \n");
			}
			break;
	}
	currState_ = nextState_;

	if (robot_->GetTime() - timeStartForAlignWithPegCommand_ > 5.0) {	// Timeout for AlignWithPegCommand
		isDone_ = true;
	}
}

bool AlignWithPegCommand::IsDone() {
	if (isDone_) {
		subscriber_->close();
	}

	return isDone_;
}

void AlignWithPegCommand::ReadFromJetson() {
	Profiler profilerFromJetson(robot_, "ReadFromJetson");
	printf("in front of read from jetson\n");

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
		printf("contents from jetson: %s\n", contents.c_str());
	} catch (const std::exception &exc) {
		printf("TRY CATCH FAILED IN READFROMJETSON\n");
		std::cerr << exc.what();
		desiredPivotDeltaAngle_ = 0.0;
		desiredDistance_ = 0.0;
	}
	printf("in end of read from jetson\n");
}

AlignWithPegCommand::~AlignWithPegCommand() { }
