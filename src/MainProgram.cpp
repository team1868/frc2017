#include <WPILib.h>
#include "RobotModel.h"
#include "Controllers/DriveController.h"
#include "Auto/AutoController.h"
#include "Auto/Modes/TestMode.h"

class MainProgram : public IterativeRobot {
	RobotModel *robot;
	DriveController *driveController;
	AutoController *autoController;
	LiveWindow *liveWindow;

	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;

public:
	void RobotInit() {
		ResetTimerVariables();
		robot = new RobotModel();
		driveController = new DriveController(robot);
		autoController = new AutoController();
		liveWindow = LiveWindow::GetInstance();
	}

	void AutonomousInit() override {
		ResetTimerVariables();
		driveController->Init();
		TestMode *pathAuto = new TestMode(driveController);
		autoController->SetAutonomousMode(pathAuto);
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		if (!autoController->IsDone()) {
			autoController->Update(currTimeSec, deltaTimeSec);
		}
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

	}

	void TestPeriodic() {

	}

private:
	void ResetTimerVariables() {
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

	void UpdateTimerVariables() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->GetTime();
		deltaTimeSec = currTimeSec - lastTimeSec;
	}
};

START_ROBOT_CLASS(MainProgram)
