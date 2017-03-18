#include "WPILib.h"
#include "RobotModel.h"
#include "Controllers/DriveController.h"
#include "Controllers/SuperstructureController.h"
#include "Auto/AutoController.h"
#include "Auto/Modes/AutoMode.h"
#include "Auto/Modes/OneGearMode.h"
#include "Auto/Modes/OneGearHighShootMode.h"
#include "Logger.h"
#include <string.h>

class MainProgram : public IterativeRobot {
	RobotModel *robot_;
	ControlBoard *humanControl_;
	DriveController *driveController_;
	SuperstructureController *superstructureController_;
	AutoController *autoController_;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;

	double currTimeSec_, lastTimeSec_, deltaTimeSec_;

//	cs::UsbCamera gearCamera;
//	cs::UsbCamera climbCamera;
//	cs::VideoSink server;
public:
	void RobotInit() {
		ResetTimerVariables();
		robot_ = new RobotModel();
		humanControl_ = new ControlBoard();

		navXSource_ = new NavXPIDSource(robot_);
		talonEncoderSource_ = new TalonEncoderPIDSource(robot_);

		driveController_ = new DriveController(robot_, humanControl_, navXSource_, talonEncoderSource_);
		superstructureController_ = new SuperstructureController(robot_, humanControl_);		// TODO
		autoController_ = new AutoController();

		robot_->ZeroNavXYaw();
		Wait(1.0);
		navXSource_->ResetAccumulatedYaw();

		//CameraServer::GetInstance()->StartAutomaticCapture();
//		gearCamera = CameraServer::GetInstance()->StartAutomaticCapture(1);		// Starting camera
//		climbCamera = CameraServer::GetInstance()->StartAutomaticCapture(0);
//		server = CameraServer::GetInstance()->GetServer();
		// TODO move this to autocontroller? meh
	}

	void AutonomousInit() {
		ResetTimerVariables();
		ResetControllers();
		robot_->SetLowGear();
		robot_->SetGearMech(true);

		int kAutoMode = robot_->pini_->geti("AUTO MODE", "autoMode", 0);
		enum autoModes {
			kBlank, kDriveStraight, kLeftLift, kMiddleLift, kRightLift, kLeftLiftAndShoot, kRightLiftAndShoot
		};

		/* ------------------ AUTO MODES ------------------ (TO IMPLEMENT SOME)
		 * 0 BLANK
		 *
		 * 1 DRIVE STRAIGHT (uses DriveStraightCommand)
		 *
		 * 2 LEFT LIFT (mp + vision)
		 * 3 MIDDLE LIFT (mp + vision)
		 * 4 RIGHT LIFT (mp + vision)
		 *
		 * 5 LEFT LIFT (mp + vision) AND SHOOT (mp + vision)
		 * 6 RIGHT LIFT (mp + vision) AND SHOOT (mp + vision)
		 * ------------------------------------------------
		 */

		AutoMode *autoMode;

		switch(kAutoMode) {
		case kBlank :
			break;
		case kDriveStraight :
			break;
		case kLeftLift :
			autoMode = new OneGearMode(robot_, navXSource_, talonEncoderSource_);
			break;
		case kMiddleLift :
			autoMode = new OneGearMode(robot_, navXSource_, talonEncoderSource_);
			break;
		case kRightLift :
			autoMode = new OneGearMode(robot_, navXSource_, talonEncoderSource_);
			break;
		case kLeftLiftAndShoot :
			autoMode = new OneGearHighShootMode(robot_, superstructureController_, navXSource_, talonEncoderSource_, true);
			break;
		case kRightLiftAndShoot :
			autoMode = new OneGearHighShootMode(robot_, superstructureController_, navXSource_, talonEncoderSource_, false);
			break;
		default :
			//autoMode = new OneGearHighShootMode(robot_, superstructureController_, navXSource_, talonEncoderSource_, true);
			break;
		}
		autoController_->SetAutonomousMode(autoMode);
		autoController_->Init();
	}

	void AutonomousPeriodic() {
		UpdateTimerVariables();
		if (!autoController_->IsDone()) {
			autoController_->Update(currTimeSec_, deltaTimeSec_);
		}
		SmartDashboard::PutNumber("NavX angle", robot_->GetNavXYaw());
		driveController_->PrintDriveValues();

		Logger::LogState(robot_);
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

		if(humanControl_->GetGearCameraDesired()) {
			SmartDashboard::PutString("Camera", "Gear");
//			server.SetSource(gearCamera);
		} else {
			SmartDashboard::PutString("Camera", "Climb");
//			server.SetSource(climbCamera);
		}
		// Logs state of robot
		Logger::LogState(robot_);
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

		SmartDashboard::PutNumber("LeftJoy Y", humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY));
		SmartDashboard::PutNumber("RightJoy X", humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX));

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
		superstructureController_->RefreshIni();
		autoController_->RefreshIni();		// TODO put in other method
	}
};

START_ROBOT_CLASS(MainProgram)
