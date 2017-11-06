#include <Auto/Commands/PathCommand.h>
#include "CANTalon.h"

const double WHEELBASE_WIDTH = 30.8 / 12.0;
const double WHEEL_DIAMETER = 3.5 / 12.0;    // in ft
const double TIME_STEP = 0.02;               // in s
const double MAX_VELOCITY = 12.0;             // in ft/s  (changed from 15) -- probably more close to 12
const double MAX_ACCELERATION = 4.0;         // in ft/(s^2)
const double MAX_JERK = 60.0;                // in ft/(s^3) (changed from 60)
const int TICKS_PER_REV = 256;

/* if starting w gear mech side (backwards):
 * +x is backwards, +y is right, +heading is counterclockwise
 * remember to negate position and velocity!

 * if starting w intake side (forwards):
 * +x is forwards, +y is left, +heading is counterclockwise
 */

PathCommand::PathCommand(RobotModel *robot, Path path) {
	robot_ = robot;
	path_ = path;

	isDone_ = false;

	p1_x_ = 0.0;
	p1_y_ = 0.0;
	p1_r_ = 0.0;

	p2_x_ = 0.0;
	p2_y_ = 0.0;
	p2_r_ = 0.0;

	p3_x_ = 0.0;
	p3_y_ = 0.0;
	p3_r_ = 0.0;

	p4_x_ = 0.0;
	p4_y_ = 0.0;
	p4_r_ = 0.0;

	pointLength_ = 0;
	trajectoryLength_ = 0;

	leftTrajectory_ = NULL;
	rightTrajectory_ = NULL;

	leftEncoderFollower_ = NULL;
	rightEncoderFollower_ = NULL;

	leftEncoderPosition_ = 0;
	rightEncoderPosition_ = 0;

	leftError = 0.0;
	rightError = 0.0;

	lastLeftDistance_ = 0.0;
	lastRightDistance_ = 0.0;

	initialAngle_ = 0.0;
}

void PathCommand::Init() {
	switch(path_) {
		case(kLeftLift) :
			printf("Left lift\n");
			p1_x_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p1_x", 0.0);
			p1_y_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p1_y", 0.0);
			p1_r_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p1_r", 0.0);

			p2_x_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p2_x", 0.0);
			p2_y_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p2_y", 0.0);
			p2_r_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p2_r", 0.0);

			p3_x_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p3_x", 0.0);
			p3_y_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p3_y", 0.0);
			p3_r_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p3_r", 0.0);

			p4_x_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p4_x", 0.0);
			p4_y_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p4_y", 0.0);
			p4_r_ = robot_->pini_->getf("LEFT LIFT WAYPOINTS", "p4_r", 0.0);

			pointLength_ = robot_->pini_->geti("LEFT LIFT WAYPOINTS", "pointLength", 0);

			break;
		case(kMiddleLift) :
			printf("Middle lift\n");

			p1_x_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p1_x", 0.0);
			p1_y_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p1_y", 0.0);
			p1_r_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p1_r", 0.0);

			p2_x_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p2_x", 0.0);
			p2_y_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p2_y", 0.0);
			p2_r_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p2_r", 0.0);

			p3_x_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p3_x", 0.0);
			p3_y_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p3_y", 0.0);
			p3_r_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p3_r", 0.0);

			p4_x_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p4_x", 0.0);
			p4_y_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p4_y", 0.0);
			p4_r_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "p4_r", 0.0);

			pointLength_ = robot_->pini_->getf("MIDDLE LIFT WAYPOINTS", "pointLength", 0.0);
			break;
		case(kRightLift) :
			printf("Right lift\n");

			p1_x_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p1_x", 0.0);
			p1_y_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p1_y", 0.0);
			p1_r_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p1_r", 0.0);

			p2_x_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p2_x", 0.0);
			p2_y_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p2_y", 0.0);
			p2_r_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p2_r", 0.0);

			p3_x_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p3_x", 0.0);
			p3_y_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p3_y", 0.0);
			p3_r_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p3_r", 0.0);

			p4_x_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p4_x", 0.0);
			p4_y_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p4_y", 0.0);
			p4_r_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "p4_r", 0.0);

			pointLength_ = robot_->pini_->getf("RIGHT LIFT WAYPOINTS", "pointLength", 0.0);
			break;
		case(kHighGoalAfterLeftLift) :
			printf("High goal after left lift\n");

			p1_x_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p1_x", 0.0);
			p1_y_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p1_y", 0.0);
			p1_r_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p1_r", 0.0);

			p2_x_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p2_x", 0.0);
			p2_y_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p2_y", 0.0);
			p2_r_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p2_r", 0.0);

			p3_x_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p3_x", 0.0);
			p3_y_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p3_y", 0.0);
			p3_r_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p3_r", 0.0);

			p4_x_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p4_x", 0.0);
			p4_y_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p4_y", 0.0);
			p4_r_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "p4_r", 0.0);

			pointLength_ = robot_->pini_->getf("LEFT HIGH GOAL WAYPOINTS", "pointLength", 0.0);
			break;
		case(kHighGoalAfterRightLift) :
			printf("High goal after right lift\n");

			p1_x_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p1_x", 0.0);
			p1_y_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p1_y", 0.0);
			p1_r_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p1_r", 0.0);

			p2_x_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p2_x", 0.0);
			p2_y_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p2_y", 0.0);
			p2_r_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p2_r", 0.0);

			p3_x_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p3_x", 0.0);
			p3_y_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p3_y", 0.0);
			p3_r_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p3_r", 0.0);

			p4_x_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p4_x", 0.0);
			p4_y_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p4_y", 0.0);
			p4_r_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "p4_r", 0.0);

			pointLength_ = robot_->pini_->getf("RIGHT HIGH GOAL WAYPOINTS", "pointLength", 0.0);
			break;
		default :
			printf("MOTION PROFILE IS NULL\n");
			break;
	}

	Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * pointLength_);
	if (pointLength_ >= 1) {
		Waypoint p1 = { p1_x_, p1_y_, d2r(p1_r_) };
		points[0] = p1;
	}

	if (pointLength_ >= 2) {
		Waypoint p2 = { p2_x_, p2_y_, d2r(p2_r_) };
		points[1] = p2;
	}

	if (pointLength_ >= 3) {
		Waypoint p3 = { p3_x_, p3_y_, d2r(p3_r_) };
		points[2] = p3;
	}

	if (pointLength_ >= 4 ) {
		Waypoint p4 = { p4_x_, p4_y_, d2r(p4_r_) };
		points[3] = p4;
	}

	TrajectoryCandidate candidate;

	// Arguments:
	// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
	// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
	//                      PATHFINDER_SAMPLES_LOW  (10 000)
	//                      PATHFINDER_SAMPLES_FAST (1 000)
	// Time Step:           0.001 Seconds
	// Max Velocity:        15 m/s
	// Max Acceleration:    10 m/s/s
	// Max Jerk:            60 m/s/s/s
	pathfinder_prepare(points, pointLength_, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, TIME_STEP, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK, &candidate);

	trajectoryLength_ = candidate.length;

	printf("trajectory length in init: %i\n", trajectoryLength_);

	Segment *trajectory = (Segment*)malloc(sizeof(Segment) * trajectoryLength_);

	pathfinder_generate(&candidate, trajectory);

	leftTrajectory_ = (Segment*)malloc(sizeof(Segment) * trajectoryLength_);
	rightTrajectory_ = (Segment*)malloc(sizeof(Segment) * trajectoryLength_);

	pathfinder_modify_tank(trajectory, trajectoryLength_, leftTrajectory_, rightTrajectory_, WHEELBASE_WIDTH);

	for (int i = 0; i < trajectoryLength_; i++) {
		cout << "position: " << trajectory->position << endl;
		cout << "velocity: " << trajectory->velocity << endl;
		cout << "heading: " << trajectory->heading << endl;
	}

	leftEncoderFollower_ = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	leftEncoderFollower_->last_error = 0; leftEncoderFollower_->segment = 0; leftEncoderFollower_->finished = 0;     // Just in case!

	rightEncoderFollower_ = (EncoderFollower*)malloc(sizeof(EncoderFollower));
	rightEncoderFollower_->last_error = 0; rightEncoderFollower_->segment = 0; rightEncoderFollower_->finished = 0;     // Just in case!

	double lPFac = robot_->pini_->getf("MOTION PROFILE PID", "lPFac", 1.0);
	double lIFac = robot_->pini_->getf("MOTION PROFILE PID", "lIFac", 0.0);
	double lDFac = robot_->pini_->getf("MOTION PROFILE PID", "lDFac", 0.0);
	double lVFac = robot_->pini_->getf("MOTION PROFILE PID", "lVFac", 1.0);
	double lAFac = robot_->pini_->getf("MOTION PROFILE PID", "lAFac", 0.0);

	double rPFac = robot_->pini_->getf("MOTION PROFILE PID", "rPFac", 1.0);
	double rIFac = robot_->pini_->getf("MOTION PROFILE PID", "rIFac", 0.0);
	double rDFac = robot_->pini_->getf("MOTION PROFILE PID", "rDFac", 0.0);
	double rVFac = robot_->pini_->getf("MOTION PROFILE PID", "rVFac", 1.0);
	double rAFac = robot_->pini_->getf("MOTION PROFILE PID", "rAFac", 0.0);

	leftEncoderPosition_ = robot_->GetDriveEncoderValue(RobotModel::kLeftWheels);

	leftEncoderConfig_ = { leftEncoderPosition_, TICKS_PER_REV, WHEEL_DIAMETER * M_PI,      // Position, Ticks per Rev, Wheel Circumference
	                         lPFac, lIFac, lDFac, lVFac / MAX_VELOCITY, lAFac};          // Kp, Ki, Kd and Kv, Ka

	rightEncoderPosition_ = robot_->GetDriveEncoderValue(RobotModel::kRightWheels);
	rightEncoderConfig_ = { rightEncoderPosition_, TICKS_PER_REV, WHEEL_DIAMETER * M_PI,      // Position, Ticks per Rev, Wheel Circumference
	                         rPFac, rIFac, rDFac, rVFac / MAX_VELOCITY, rAFac};          // Kp, Ki, Kd and Kv, Ka

	// To make sure SRX's encoder is updating the RoboRIO fast enough
	robot_->leftMaster_->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 20);
	robot_->leftSlave_->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 20);
	robot_->rightMaster_->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 20);
	robot_->rightSlave_->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 20);

	FILE *fp_traj = fopen("/home/lvuser/trajectory.csv", "w");
	pathfinder_serialize_csv(fp_traj, trajectory, trajectoryLength_);
	fclose(fp_traj);

	FILE *fp_leftTraj = fopen("/home/lvuser/left_trajectory.csv", "w");
	pathfinder_serialize_csv(fp_leftTraj, leftTrajectory_, trajectoryLength_);
	fclose(fp_leftTraj);

	FILE *fp_rightTraj = fopen("/home/lvuser/right_trajectory.csv", "w");
	pathfinder_serialize_csv(fp_rightTraj, rightTrajectory_, trajectoryLength_);
	fclose(fp_rightTraj);

	initialAngle_ = robot_->GetNavXYaw();

	printf("AT THE END OF PATH COMMAND INIT\n");
}

void PathCommand::Update(double currTimeSec, double deltaTimeSec) {
	leftEncoderPosition_ = robot_->GetDriveEncoderValue(RobotModel::kLeftWheels);
	rightEncoderPosition_ = robot_->GetDriveEncoderValue(RobotModel::kRightWheels);

	leftError = get_error(leftEncoderFollower_);
	rightError = get_error(rightEncoderFollower_);
	SmartDashboard::PutNumber("Left Error", leftError);
	SmartDashboard::PutNumber("Right Error", rightError);

	// Arg 1: The EncoderConfig
	// Arg 2: The EncoderFollower for this side
	// Arg 3: The Trajectory generated from `pathfinder_modify_tank`
	// Arg 4: The Length of the Trajectory (length used in Segment seg[length];)
	// Arg 5: The current value of your encoder

	printf("IN PATH UPDATE!!!!!\n");
	printf("trajectory length in update: %i\n", trajectoryLength_);
	printf("leftEncoderPosition: %i\n", leftEncoderPosition_);
	printf("rightEncoderPosition: %i\n", rightEncoderPosition_);

	double l = pathfinder_follow_encoder(leftEncoderConfig_, leftEncoderFollower_, leftTrajectory_, trajectoryLength_, leftEncoderPosition_);
	double r = pathfinder_follow_encoder(rightEncoderConfig_, rightEncoderFollower_, rightTrajectory_, trajectoryLength_, rightEncoderPosition_);

	// -- using l and r from the previous code block -- //
	double gyro_heading = robot_->GetNavXYaw() - initialAngle_;

	double desired_heading = r2d(leftEncoderFollower_->heading);
	if (desired_heading >= 180.0) {
		desired_heading -= 360.0;
	}

	double angle_difference = desired_heading - gyro_heading;    // Make sure to bound this from -180 to 180, otherwise you will get super large values
	double turn = 1.0 * (1.0/80.0) * angle_difference;

	SmartDashboard::PutNumber("Left output", l);
	SmartDashboard::PutNumber("Right output", r);

	robot_->SetDriveValues(RobotModel::kLeftWheels, l);
	robot_->SetDriveValues(RobotModel::kRightWheels, r);

	if (!logData_.is_open()) {
		logData_.open(Logger::GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_moprolog.csv")).c_str()), std::ofstream::out | std::ofstream::app);
		logData_ << "Time, DeltaTime, LeftEncoderValue, RightEncoderValue, LeftDistance, RightDistance, LeftExpectedDistance, RightExpectedDistance, LeftVelocity, Right Velocity, "
				 << "LeftExpectedVelocity, RightExpectedVelocity, LeftError, RightError, LeftOutput, RightOutput, Turn, NavXAngle, ExpectedHeading, ExpectedHeading_Edited" << "\r\n";
	}

	logData_ << robot_->GetTime() << ", " <<
			   deltaTimeSec << ", " <<
			   robot_->GetDriveEncoderValue(RobotModel::kLeftWheels) << ", " <<
			   robot_->GetDriveEncoderValue(RobotModel::kRightWheels) << ", " <<
			   robot_->GetLeftDistance() << ", " <<
			   robot_->GetRightDistance() << ", " <<
			   get_expected_position(leftEncoderFollower_) << ", " <<
			   get_expected_position(rightEncoderFollower_) << ", " <<
			   (robot_->GetLeftDistance() - lastLeftDistance_) / deltaTimeSec << ", " <<
			   (robot_->GetRightDistance() - lastRightDistance_) / deltaTimeSec << ", " <<
			   get_expected_velocity(leftEncoderFollower_) << ", " <<
			   get_expected_velocity(rightEncoderFollower_) << ", " <<
			   leftError << ", " <<
			   rightError << ", " <<
			   l << ", " <<
			   r << ", " <<
			   turn << ", " <<
			   gyro_heading << ", " <<
			   r2d(leftEncoderFollower_->heading) << ", " <<
			   desired_heading << "\r\n";

	logData_.flush();

	lastLeftDistance_ = robot_->GetLeftDistance();
	lastRightDistance_ = robot_->GetRightDistance();
}

bool PathCommand::IsDone() {
	if ((leftEncoderFollower_->finished == 1) && (rightEncoderFollower_->finished == 1)) {
		printf("DONE WITH PATH COMMAND\n");
		robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
		robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);

		isDone_ = true;

		// Free memory for the followers and trajectories
		free(leftEncoderFollower_);
		free(rightEncoderFollower_);
		free(leftTrajectory_);
		free(rightTrajectory_);

		return true;
	} else {
		isDone_ = false;
		return false;
	}
}

PathCommand::~PathCommand() {

}
