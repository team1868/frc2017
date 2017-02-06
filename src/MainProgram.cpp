#include <Auto/Modes/OneGearMode.h>
#include <WPILib.h>
#include "RobotModel.h"
#include "Controllers/DriveController.h"
#include "Controllers/SuperstructureController.h"
#include "Auto/AutoController.h"

class MainProgram : public IterativeRobot {
	RobotModel *robot_;
	ControlBoard *humanControl_;
	DriveController *driveController_;
	SuperstructureController *superstructureController_;
	AutoController *autoController_;
	LiveWindow *liveWindow_;
	OneGearMode *liftOneMode;	// move this later

	double currTimeSec_;
	double lastTimeSec_;
	double deltaTimeSec_;

public:
	void RobotInit() {
		ResetTimerVariables();
		robot_ = new RobotModel();
		humanControl_ = new ControlBoard();
		driveController_ = new DriveController(robot_, humanControl_);
		superstructureController_ = new SuperstructureController();		// TODO
		//autoController_ = new AutoController(robot_, driveController_, superstructureController_, humanControl_);
	}


	void AutonomousInit() {
		ResetTimerVariables();
		ResetControllers();
//		autoController_->SetAutonomousMode(liftOneMode);
//		autoController_->Init();
//		liftOneMode = new OneGearMode();
		liftOneMode = new OneGearMode(robot_);
		liftOneMode->Init();
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		if (!liftOneMode->IsDone()) {
			liftOneMode->Update(currTimeSec_, deltaTimeSec_);
		}
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
//		autoController_->Reset();
		driveController_->Reset();
		superstructureController_->Reset();
	}
};

START_ROBOT_CLASS(MainProgram)
