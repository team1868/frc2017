#include <Auto/Commands/PathCommand.h>

PathCommand::PathCommand(DriveController *myDriveController, Waypoint *myPoints) {
	driveController = myDriveController;
	points = myPoints;
}

void PathCommand::Init() {
	int POINT_LENGTH = 3;	// TODO change this??
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &trajectoryCandidate);
	int length = trajectoryCandidate.length;
	Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));	// Array of segments (the trajectory points) to store the trajectory in
	int result = pathfinder_generate(&trajectoryCandidate, trajectory);	// Generates the trajectory
	if (result < 0) {	// An error occurred
	    printf("Uh-Oh! Trajectory could not be generated!\n");
	}
	free(trajectory);
}

void PathCommand::Update(double currTimeSec, double deltaTimeSec) {
	driveController->FollowTrajectory(trajectoryCandidate);
}

bool PathCommand::IsDone() {
	return driveController->IsDoneFollowingTrajectory();
}

PathCommand::~PathCommand() {

}
