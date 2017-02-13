#include <Auto/Commands/AlignWithPegCommand.h>

using namespace std;

bool global_pivotCommandIsDone;

AlignWithPegCommand::AlignWithPegCommand(RobotModel *robot) {
	// TODO Auto-generated constructor stub
	context_ = new zmq::context_t(1);
	subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
	subscriber_->connect("tcp://10.18.68.29:5563");
	subscriber_->setsockopt( ZMQ_SUBSCRIBE, "B", 1);

	robot_ = robot;

	navxSource_ = new NavxPIDSource(robot_);
	talonEncoderSource_ = new TalonEncoderPIDSource(robot_);
	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();
	pivotCommand_ = NULL;

	driveStraightCommand_ = new DriveStraightCommand(navxSource_, talonEncoderSource_, angleOutput_, distanceOutput_,
			robot_, 3.0);

	isDone_ = false;
	//initializedPivotCommand_ = false;
	pivotCommandIsDone_ = true;
	driveStraightCommandIsDone_ = false;

	pivotDeltaAngle_ = 0.0;
}

void AlignWithPegCommand::RefreshIni() {
	//pivotCommand_->RefreshIni();
}

void AlignWithPegCommand::Init() {
	pivotCommandIsDone_ = true;
	driveStraightCommandIsDone_ = false;
	pivotDeltaAngle_ = 0.0;
	isDone_ = false;

	driveStraightCommand_->Init();
}

void AlignWithPegCommand::Update(double currTimeSec, double deltaTimeSec) {
	/*
	string address = s_recv (*subscriber_);
	string contents = s_recv (*subscriber_);

	pivotDeltaAngle_ = stod(contents);

	if (pivotCommandIsDone_) {
		if (fabs(pivotDeltaAngle_) < 1.0) {
			isDone_ = true;
		} else {
			SmartDashboard::PutNumber("Initial navx angle", robot_->GetNavxYaw());
			SmartDashboard::PutNumber("Pivot delta angle", pivotDeltaAngle_);
			// -pivotDeltaAngle_ because Jetson returns positive angles to the right
			pivotCommand_ = new PivotCommand(robot_, -pivotDeltaAngle_, false, navxSource_);
			pivotCommand_->Init();
			pivotCommandIsDone_ = false;
		}
	} else {
		pivotCommand_->Update(currTimeSec, deltaTimeSec);
		if (global_pivotCommandIsDone) {
		//if (pivotCommand_->IsDone()) {
			pivotCommandIsDone_ = true;
//			std::terminate();
			isDone_ = true;	// TAKE OUT LATER
		}
	}

*/
	if (!driveStraightCommand_->IsDone()) {
		driveStraightCommand_->Update(0.0, 0.0); 	// add timer later
	} else {
		driveStraightCommandIsDone_   = true;
		isDone_ = true;
	}

}

bool AlignWithPegCommand::IsDone() {
	return isDone_;		// TODO change
}

AlignWithPegCommand::~AlignWithPegCommand() {
	// TODO Auto-generated destructor stub
}
