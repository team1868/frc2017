#include <Auto/Commands/PathCommand.h>

PathCommand::PathCommand(DriveController *driveController, Waypoint *points, int pointLength) {
	driveController_ = driveController;
	points_ = points;	// Takes ownership of memory
	pointLength_ = pointLength;
}

//int length;

void PathCommand::Init() {
	TrajectoryCandidate trajectoryCandidate;
	pathfinder_prepare(points_, pointLength_, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.01, 11.0, 10.0, 60.0, &trajectoryCandidate);	//TODO verify arguments
	int length = trajectoryCandidate.length;
	Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));	// Array of segments (the trajectory points) to store the trajectory in
	int result = pathfinder_generate(&trajectoryCandidate, trajectory);	// Generates the trajectory
	if (result < 0) {	// An error occurred
	    printf("Uh-Oh! Trajectory could not be generated!\n");
	}
	leftTrajectory_ = (Segment*)malloc(sizeof(Segment) * length);
	rightTrajectory_ = (Segment*)malloc(sizeof(Segment) * length);

	double wheelbase_width = 0.6;			// TODO CHECK THIS
	pathfinder_modify_tank(trajectory, length, leftTrajectory_, rightTrajectory_, wheelbase_width);
	free(trajectory);

	driveController_->SetupTrajectory(leftTrajectory_, rightTrajectory_, length);
}

void PathCommand::Update(double currTimeSec, double deltaTimeSec) {
	//driveController_->Update(currTimeSec, deltaTimeSec);
	//driveController_->SetupTrajectory(leftTrajectory_, rightTrajectory_, length);
	driveController_->UpdateMotionProfile();
}

bool PathCommand::IsDone() {
	return driveController_->IsDone();
}

PathCommand::~PathCommand() {
	free(leftTrajectory_);
	free(rightTrajectory_);
	free(points_);
}
