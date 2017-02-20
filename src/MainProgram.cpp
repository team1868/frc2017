#include <Auto/Modes/OneGearMode.h>
#include <Auto/Modes/TestMode.h>
#include <WPILib.h>
#include "RobotModel.h"
#include "Controllers/DriveController.h"
#include "Controllers/SuperstructureController.h"
#include "Auto/AutoController.h"

class MainProgram : public IterativeRobot {
	RobotModel *robot_; /**< Testing. */
	ControlBoard *humanControl_;
	DriveController *driveController_;
	SuperstructureController *superstructureController_;
	AutoController *autoController_;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;

	LiveWindow *liveWindow_;

	double currTimeSec_;
	double lastTimeSec_;
	double deltaTimeSec_;

public:
	void RobotInit() {
		ResetTimerVariables();
		robot_ = new RobotModel();
		humanControl_ = new ControlBoard();
		driveController_ = new DriveController(robot_, humanControl_);
		superstructureController_ = new SuperstructureController(robot_, humanControl_);		// TODO
		autoController_ = new AutoController();

		navXSource_ = new NavXPIDSource(robot_);
		talonEncoderSource_ = new TalonEncoderPIDSource(robot_);

		Wait(1.0);
		robot_->ZeroNavXYaw();
		Wait(1.0);
		navXSource_->ResetAccumulatedYaw();		// TODO reset accumulated yaw at some point
	}

	void AutonomousInit() {
		ResetTimerVariables();
		ResetControllers();
		OneGearMode *liftTwoMode = new OneGearMode(robot_, navXSource_, talonEncoderSource_);
		autoController_->SetAutonomousMode(liftTwoMode);
		autoController_->Init();
	}

	void AutonomousPeriodic() {
		robot_->SetGearMechOut();
		UpdateTimerVariables();
		if (!autoController_->IsDone()) {
			autoController_->Update(currTimeSec_, deltaTimeSec_);
		} else {
			SmartDashboard::PutString("Auto Mode", "Done");
		}
		SmartDashboard::PutNumber("NavX angle", robot_->GetNavXYaw());
		driveController_->PrintDriveValues();
	}

	void TeleopInit() {
		ResetControllers();
		robot_->SetHighGear();
	}

	void TeleopPeriodic() {
		humanControl_->ReadControls();
		driveController_->Update(currTimeSec_, deltaTimeSec_);
		superstructureController_->Update(currTimeSec_, deltaTimeSec_);
	}

	void TestInit() {
		ResetControllers();
	}

	void TestPeriodic() {
	}

	void DisabledPeriodic() {
		SmartDashboard::PutNumber("NavX angle", robot_->GetNavXYaw());
		robot_->SetPercentVBusDrive();
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
		robot_->ClearMotionProfileTrajectories();
		driveController_->PrintDriveValues();
	}

private:
	void ResetTimerVariables() {
		currTimeSec_ = 0.0;
		lastTimeSec_ = 0.0;
		deltaTimeSec_ = 0.0;
	}

	void UpdateTimerVariables() {
		lastTimeSec_ = currTimeSec_;
		currTimeSec_ = robot_->GetTime();
		deltaTimeSec_ = currTimeSec_ - lastTimeSec_;
	}

	void ResetControllers() {
		autoController_->Reset();
		driveController_->Reset();
		superstructureController_->Reset();

		robot_->RefreshIni();	// TODO move
		autoController_->RefreshIni();		// TODO put in other method
	}
};

START_ROBOT_CLASS(MainProgram)
