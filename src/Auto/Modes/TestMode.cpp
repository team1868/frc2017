#include "Auto/Modes/TestMode.h"

TestMode::TestMode(DriveController *driveController) {
	int POINT_LENGTH = 3;
	Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);
//	Waypoint p1 = { -4, -1, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
//	Waypoint p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
//	Waypoint p3 = {  2, 4, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians

	// Drives forward 2 feet
	Waypoint p1 = { 0, 0, 0};
    Waypoint p2 = { 1, 0, 0};
    Waypoint p3 = { 2, 0, 0};

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	testPath_ = new PathCommand(driveController, points, POINT_LENGTH);	// Hands over control of points memory to PathCommand
}

void TestMode::Init() {
	testPath_->Init();
}

void TestMode::Update(double currTimeSec, double deltaTimeSec) {
	testPath_->Update(currTimeSec, deltaTimeSec);
	printf("in test path update\n");
}

bool TestMode::IsDone() {
	return testPath_->IsDone();
}

TestMode::~TestMode() {

}
