#include <Auto/Modes/OneGearMode.h>
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

	NavxPIDSource *navxSource_;
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
		//autoController_ = new AutoController(robot_, driveController_, superstructureController_, humanControl_);
		autoController_ = new AutoController();

		navxSource_ = new NavxPIDSource(robot_);
		talonEncoderSource_ = new TalonEncoderPIDSource(robot_);

//		navxSource_ = new NavXPIDSource(robot_);
		Wait(1.0);
		robot_->ZeroNavxYaw();
//		Wait(1.0);
//		navxSource_->ResetAccumulatedYaw();		// TODO reset accumulated yaw at some point
	}

	void AutonomousInit() {
		ResetTimerVariables();
		ResetControllers();
		OneGearMode *liftTwoMode = new OneGearMode(robot_, navxSource_, talonEncoderSource_);	// TODO make this take in DriveController

		autoController_->SetAutonomousMode(liftTwoMode);
		autoController_->Init();
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		if (!autoController_->IsDone()) {
			autoController_->Update(currTimeSec_, deltaTimeSec_);
		}
		SmartDashboard::PutNumber("Navx angle", robot_->GetNavxYaw());
	}

	void TeleopInit() {
		ResetControllers();
	}

	void TeleopPeriodic() {
		humanControl_->ReadControls();
		driveController_->Update(currTimeSec_, deltaTimeSec_);
	}

	void TestInit() {
		ResetControllers();
	}

	void TestPeriodic() {
		driveController_->PrintDriveValues();
	}

	void DisabledPeriodic() {
		SmartDashboard::PutNumber("Navx angle", robot_->GetNavxYaw());
		robot_->SetPercentVDrive();
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
//		talon_.SetControlMode(CANTalon::kPercentVbus);
//		talon_.Set( 0 );
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
