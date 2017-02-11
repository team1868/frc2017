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

	pivotCommand_ = NULL;
	navxSource_ = new NavxPIDSource(robot_);

	isDone_ = false;
	//initializedPivotCommand_ = false;
	pivotCommandIsDone_ = true;

	pivotDeltaAngle_ = 0.0;
}

void AlignWithPegCommand::RefreshIni() {
	//pivotCommand_->RefreshIni();
}

void AlignWithPegCommand::Init() {
	pivotCommandIsDone_ = true;
	pivotDeltaAngle_ = 0.0;
	isDone_ = false;
}

void AlignWithPegCommand::Update(double currTimeSec, double deltaTimeSec) {
	string address = s_recv (*subscriber_);
	//  Read message contents
	string contents = s_recv (*subscriber_);

//	cout << contents << endl;

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

//	this_thread::sleep_for(chrono::milliseconds(20));

	//SmartDashboard::PutNumber("Yaw", robot_->GetNavxYaw());
	//SmartDashboard::PutNumber("Accumulated yaw", navxSource_->PIDGet());
}

bool AlignWithPegCommand::IsDone() {
	return isDone_;		// TODO change
}

AlignWithPegCommand::~AlignWithPegCommand() {
	// TODO Auto-generated destructor stub
}
