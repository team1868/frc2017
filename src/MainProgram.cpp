#include "WPILib.h"
#include "RobotModel.h"
#include "Controllers/DriveController.h"
#include "Controllers/SuperstructureController.h"
#include "Auto/AutoController.h"
#include "Auto/Modes/OneGearMode.h"
#include "Auto/Modes/OneGearHighShootMode.h"

class MainProgram : public IterativeRobot {
	RobotModel *robot_;
	ControlBoard *humanControl_;
	DriveController *driveController_;
	SuperstructureController *superstructureController_;
	AutoController *autoController_;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;

	double currTimeSec_, lastTimeSec_, deltaTimeSec_;

public:
	void RobotInit() {
		ResetTimerVariables();
		robot_ = new RobotModel();
		humanControl_ = new ControlBoard();
		driveController_ = new DriveController(robot_, humanControl_, navXSource_);
		superstructureController_ = new SuperstructureController(robot_, humanControl_);		// TODO
		autoController_ = new AutoController();

		navXSource_ = new NavXPIDSource(robot_);
		talonEncoderSource_ = new TalonEncoderPIDSource(robot_);

		robot_->ZeroNavXYaw();
		Wait(1.0);
		navXSource_->ResetAccumulatedYaw();
	}

	// TODO PUT INI FILE HERE
	// TODO switches for auto
	void AutonomousInit() {
		ResetTimerVariables();
		ResetControllers();
		robot_->SetLowGear();
		OneGearHighShootMode *oneGearHighShootMode = new OneGearHighShootMode(robot_, superstructureController_, navXSource_, talonEncoderSource_);
		autoController_->SetAutonomousMode(oneGearHighShootMode);
		autoController_->Init();
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		robot_->SetGearMechOut();
		if (!autoController_->IsDone()) {
			autoController_->Update(currTimeSec_, deltaTimeSec_);
		}
		SmartDashboard::PutNumber("NavX angle", robot_->GetNavXYaw());
		driveController_->PrintDriveValues();
	}

	void TeleopInit() {
		ResetControllers();
		robot_->SetHighGear();
		robot_->SetPercentVBusDriveMode();		// THIS SHOULD ALREADY BE IN RESET CONTROLLERS
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
		SmartDashboard::PutNumber("Flywheel Velocity", robot_->GetFlywheelEncoder()->GetRate());
		SmartDashboard::PutNumber("Flywheel Distance", robot_->GetFlywheelEncoder()->GetDistance());
		SmartDashboard::PutNumber("Flywheel Pulses", robot_->GetFlywheelEncoder()->GetRaw());
	}

	void DisabledPeriodic() {
		robot_->SetPercentVBusDriveMode();
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
		robot_->ClearMotionProfileTrajectories();
		driveController_->PrintDriveValues();
		SmartDashboard::PutNumber("Flywheel Velocity", robot_->GetFlywheelEncoder()->GetRate());
		SmartDashboard::PutNumber("Flywheel Distance", robot_->GetFlywheelEncoder()->GetDistance());
		SmartDashboard::PutNumber("Flywheel Pulses", robot_->GetFlywheelEncoder()->GetRaw());
		SmartDashboard::PutNumber("Flywheel Dial Value", humanControl_->GetFlywheelVelAdjust());

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
		RefreshIni();
	}

	void RefreshIni() {
		robot_->RefreshIni();	// TODO move
		autoController_->RefreshIni();		// TODO put in other method
	}
};

START_ROBOT_CLASS(MainProgram)
