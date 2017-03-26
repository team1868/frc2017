#include <Auto/Commands/AlignWithPegCommand.h>

using namespace std;

AlignWithPegCommand::AlignWithPegCommand(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource) {
	printf("in beginning of alignwithpegcommand\n");

	visionLog_.open("/home/lvuser/vision_log.csv");

	/*
	angleContext_ = new zmq::context_t(1);
	distanceContext_ = new zmq::context_t(1);

	angleSubscriber_ = new zmq::socket_t(*angleContext_, ZMQ_SUB);
	angleSubscriber_->connect("tcp://10.18.68.15:5563");	// MAKE SURE RIGHT IP

	distanceSubscriber_ = new zmq::socket_t(*distanceContext_, ZMQ_SUB);
	distanceSubscriber_->connect("tcp://10.18.68.15:5563");	// MAKE SURE RIGHT IP
	angleSubscriber_->setsockopt( ZMQ_SUBSCRIBE, "ANGLE", 1);
	distanceSubscriber_->setsockopt( ZMQ_SUBSCRIBE, "DISTANCE", 1);
	*/

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
//	currState_ = kDriveStraightInit;
//	nextState_ = kDriveStraightInit;


	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	numTimesInkPivotToAngleInit = 0;
	numTimesInkDriveStraightInit = 0;

	printf("in alignwithpegcommand init\n");
}

void AlignWithPegCommand::Update(double currTimeSec, double deltaTimeSec) {
/*
	string angleAddress = s_recv (*angleSubscriber_);
	string angleContents = s_recv (*angleSubscriber_);

	string distanceAddress = s_recv (*distanceSubscriber_);
	string distanceContents = s_recv (*distanceSubscriber_);
*/

	switch (currState_) {
		case (kPivotToAngleInit) :

			printf("In kPivotToAngleInit\n");
/*			// Get angle from Jetson
			if (angleAddress == "ANGLE") {
				desiredPivotDeltaAngle_ = stod(angleContents);
				printf("ANGLE: %f\n", desiredPivotDeltaAngle_);
			} else {
				printf("angle address: %s\n", angleAddress.c_str());
			}
*/
			ReadUpdateFromJetson();

			if (fabs(desiredPivotDeltaAngle_) > 3.0) {
//			if (numTimesInkPivotToAngleInit < 3) {
//				numTimesInkPivotToAngleInit++;
//				nextState_ = kPivotToAngleInit;
//			} else if (fabs(desiredPivotDeltaAngle_) > 3.0) {		// 2 degree threshold
				// CHECK -DESIREDPIVOTDELTAANGLE_
				// negative because jetson returns angle wrong way
				printf("ANGLE FOR PIVOT COMMAND: %f\n", -desiredPivotDeltaAngle_);
				pivotCommand_ = new PivotCommand(robot_, -desiredPivotDeltaAngle_, false, navXSource_);
				pivotCommand_->Init();
				nextState_ = kPivotToAngleUpdate;
			} else {
				//nextState_ = kDriveStraightInit;
				isDone_ = true;
			}
			break;

		case (kPivotToAngleUpdate) :
			printf("in kPivotToAngleUpdate\n");
			if (!pivotCommand_->IsDone()) {
				pivotCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kPivotToAngleUpdate;
			} else {
//				nextState_ = kDriveStraightInit;
				isDone_ = true;
			}
			break;

		case (kDriveStraightInit) :
//			// Get distance from Jetson
//			if (distanceAddress == "DISTANCE") {
//				desiredDistance_ = stod(distanceContents);		// IN INCHES
//				printf("DISTANCE: %f\n", desiredDistance_);
//			}
//
//			if (numTimesInkDriveStraightInit < 3) {
//				numTimesInkDriveStraightInit++;
//				nextState_ = kDriveStraightInit;
//			} else if (fabs(desiredDistance_) > 2.0/12.0) {	// 2 in threshold
//				printf("DISTANCE FOR COMMAND: %f\n", desiredDistance_);
//				// Jetson returns in inches, so /12.0
//				// Subtract 10 inches bc of length of peg
//				// negative because technically driving backwards
//				driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_,
//						robot_, -(desiredDistance_ - 10.0)/12.0);	// converting to feet
//				driveStraightCommand_->Init();
//				nextState_ = kDriveStraightUpdate;
//			} else {
//				isDone_ = true;
//			}
			break;

		case (kDriveStraightUpdate) :
//			if (!driveStraightCommand_->IsDone()) {
//				driveStraightCommand_->Update(0.0, 0.0); 	// add timer later
//				nextState_ = kDriveStraightUpdate;
//			} else {
//				isDone_ = true;
//				// no next state
//			}
			break;
	}
	currState_ = nextState_;
}

bool AlignWithPegCommand::IsDone() {
	return isDone_;
}

void AlignWithPegCommand::ReadUpdateFromJetson() {
	// GET LAST LINE
	string lastLine;
	if (visionLog_.is_open()) {
		visionLog_.seekg(-1, ios_base::end); // go to one spot before the EOF

		bool keepLooping = true;
		while (keepLooping) {
			char ch;
			visionLog_.get(ch);                   // Get current byte's data

			if ((int) visionLog_.tellg() <= 1) { // If the data was at or before the 0th byte
				visionLog_.seekg(0);      // The first line is the last line
				keepLooping = false;                // So stop there
			} else if (ch == '\n') {            // If the data was a newline
				keepLooping = false;        // Stop at the current position.
			} else {  // If the data was neither a newline nor at the 0 byte
				visionLog_.seekg(-2, ios_base::cur); // Move to the front of that data, then to the front of the data before it
			}
		}

		getline(visionLog_, lastLine);              // Read the current line
		cout << "Result: " << lastLine << '\n';     // Display it

		visionLog_.close();
	} else {
		// ERROR
	}

	// PARSING STUFF HERE
	stringstream ss(lastLine);
	vector<string> result;

	while(ss.good()) {
		string substr;
		getline( ss, substr, ',' );
		result.push_back( substr );
	}

	double duration = stod(result.at(0));		// do something w this
	desiredPivotDeltaAngle_ = stod(result.at(1));
	desiredDistance_ = stod(result.at(2));
}

AlignWithPegCommand::~AlignWithPegCommand() {

}
