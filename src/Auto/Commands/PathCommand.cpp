#include <Auto/Commands/PathCommand.h>

PathCommand::PathCommand(DriveController *myDriveController, Waypoint *myPoints, int myPointLength) {
	driveController = myDriveController;
	points = myPoints;
	pointLength = myPointLength;
}

void PathCommand::Init() {
	TrajectoryCandidate trajectoryCandidate;
	pathfinder_prepare(points, pointLength, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &trajectoryCandidate);	//TODO verify arguments
	int length = trajectoryCandidate.length;
	Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));	// Array of segments (the trajectory points) to store the trajectory in
	int result = pathfinder_generate(&trajectoryCandidate, trajectory);	// Generates the trajectory
	if (result < 0) {	// An error occurred
	    printf("Uh-Oh! Trajectory could not be generated!\n");
	}
	leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
	rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

	double wheelbase_width = 0.6;			// TODO CHECK THIS
	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);
	free(trajectory);

	driveController->SetupTrajectory(leftTrajectory, rightTrajectory);
}

void PathCommand::Update(double currTimeSec, double deltaTimeSec) {
	driveController->Update(currTimeSec, deltaTimeSec);
}

bool PathCommand::IsDone() {
	return driveController->IsDone();
}

PathCommand::~PathCommand() {
	free(leftTrajectory);
	free(rightTrajectory);
}
