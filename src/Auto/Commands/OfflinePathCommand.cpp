/*
 * OfflinePathCommand.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: Lynn D
 */

#include <Auto/Commands/OfflinePathCommand.h>

OfflinePathCommand::OfflinePathCommand(DriveController *driveController, Waypoint *points, int pointLength) {
	driveController_ = driveController;
	points_ = points;	// Takes ownership of memory
	pointLength_ = pointLength;
}

void OfflinePathCommand::FillTrajectory(const double motionProfile[][3], Segment *trajectory, int length) {
	for (int i = 0; i < length; i++) {
		trajectory[i].position = motionProfile[i][0];
		trajectory[i].velocity = motionProfile[i][1];
		trajectory[i].dt = motionProfile[i][2];
	}
}

void OfflinePathCommand::Init() {
	int length = kLeftMotionProfileSz;
	leftTrajectory_ = (Segment*)malloc(sizeof(Segment) * length);
	rightTrajectory_ = (Segment*)malloc(sizeof(Segment) * length);

	FillTrajectory(kLeftMotionProfile, leftTrajectory_, length);
	FillTrajectory(kRightMotionProfile, rightTrajectory_, length);

	free(leftTrajectory_);
	free(rightTrajectory_);

	driveController_->SetupTrajectory(leftTrajectory_, rightTrajectory_, length);
}

void OfflinePathCommand::Update(double currTimeSec, double deltaTimeSec) {
	driveController_->Update(currTimeSec, deltaTimeSec);
}

bool OfflinePathCommand::IsDone() {
	return driveController_->IsDone();
}

OfflinePathCommand::~OfflinePathCommand() {
	free(leftTrajectory_);
	free(rightTrajectory_);
	free(points_);
}
