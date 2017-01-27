#include <WPILib.h>
#include "RobotModel.h"
#include "Controllers/DriveController.h"
#include "Auto/AutoController.h"
#include "Auto/Modes/TestMode.h"

class MainProgram : public IterativeRobot {
	RobotModel *robot_;
	ControlBoard *humanControl_;
	DriveController *driveController_;
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
		humanControl_->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kX);
		driveController_ = new DriveController(robot_, humanControl_);
		autoController_ = new AutoController();
		liveWindow_ = LiveWindow::GetInstance();
	}

	void AutonomousInit() {
		ResetTimerVariables();
		driveController_->Init();
		TestMode *pathAuto = new TestMode(driveController_);
		autoController_->SetAutonomousMode(pathAuto);
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		if (!autoController_->IsDone()) {
			autoController_->Update(currTimeSec_, deltaTimeSec_);
		}
	}

	void TeleopInit() {
		driveController_->Init();
	}

	void TeleopPeriodic() {
		humanControl_->ReadControls();
		driveController_->Update(currTimeSec_, deltaTimeSec_);
	}

	void TestInit() {
	}

	void TestPeriodic() {
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
};

START_ROBOT_CLASS(MainProgram)
