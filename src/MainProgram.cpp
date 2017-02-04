#include <WPILib.h>
#include "RobotModel.h"
#include "Controllers/DriveController.h"
#include "Controllers/SuperstructureController.h"
#include "Auto/AutoController.h"
#include "Auto/Modes/TestMode.h"

class MainProgram : public IterativeRobot {
	RobotModel *robot_;
	ControlBoard *humanControl_;
	DriveController *driveController_;
	SuperstructureController *superstructureController_;
	AutoController *autoController_;
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
		superstructureController_ = new SuperstructureController();		// TODO
		autoController_ = new AutoController();
		liveWindow_ = LiveWindow::GetInstance();
	}

	void AutonomousInit() {
		ResetTimerVariables();
		ResetControllers();
		TestMode *pathAuto = new TestMode(driveController_);
		autoController_->SetAutonomousMode(pathAuto);
		autoController_->Init();
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		if (!autoController_->IsDone()) {
			autoController_->Update(currTimeSec_, deltaTimeSec_);
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
		autoController_->Reset();
		driveController_->Reset();
		superstructureController_->Reset();
	}
};

START_ROBOT_CLASS(MainProgram)
