/*
 * MotionProfileExample.h
 *
 *  Created on: Jan 12, 2017
 *      Author: maggiewang
 */

#ifndef SRC_CONTROLLERS_MOTIONPROFILEEXAMPLE_H_
#define SRC_CONTROLLERS_MOTIONPROFILEEXAMPLE_H_

#include "WPILib.h"
#include "CANTalon.h"
#include "Controllers/Instrumentation.h"
#include <pathfinder/pathfinder.h>
#include <math.h>

// TODO edit to fit style guidelines (methods are UpperCamelCase, private variables are variable_, etc.)

const double WHEEL_DIAMETER = 7.5 / 12.0; // Feet
const double SECONDS_PER_MINUTE = 60.0;

class MotionProfileExample {
public:
	CANTalon::MotionProfileStatus _status;
	CANTalon & _talon;
	int _state = 0;
	int _loopTimeout = -1;
	bool _bStart = false;
	bool _isDone = true;
	CANTalon::SetValueMotionProfile _setValue = CANTalon::SetValueMotionProfileDisable;
	static const int kMinPointsInTalon = 5;
	static const int kNumLoopsTimeout = 10;

	void PeriodicTask() {
		_talon.ProcessMotionProfileBuffer();
	}

	Notifier _notifer;

	MotionProfileExample(CANTalon & talon) : _talon(talon), _notifer(&MotionProfileExample::PeriodicTask, this) {
		_talon.ChangeMotionControlFramePeriod(5);
		_notifer.StartPeriodic(0.005);
	}

	void reset() {
		_talon.ClearMotionProfileTrajectories();
		_setValue = CANTalon::SetValueMotionProfileDisable;
		_state = 0;
		_loopTimeout = -1;
		_bStart = false;
	}

	void control() {
		_talon.GetMotionProfileStatus(_status);

		if (_loopTimeout < 0) {
			/* do nothing, timeout is disabled */
		} else {
			if (_loopTimeout == 0) {
				instrumentation::OnNoProgress();
			} else {
				--_loopTimeout;
			}
		}

		if(_talon.GetControlMode() != CANSpeedController::kMotionProfile){
			_state = 0;
		} else {
			switch (_state) {
				case 0:
					if (_bStart) {
						_bStart = false;

						_setValue = CANTalon::SetValueMotionProfileDisable;
						startFilling(trajectory_, trajectoryLength_);	// moved startFilling() from control() to transfer the Segment array and length into startFilling()
						_state = 1;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 1:
					if (_status.btmBufferCnt > kMinPointsInTalon) {
						_setValue = CANTalon::SetValueMotionProfileEnable;
						_state = 2;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 2:
					if (_status.isUnderrun == false) {
						_loopTimeout = kNumLoopsTimeout;
					}
					if (_status.activePointValid && _status.activePoint.isLastPoint) {
						_setValue = CANTalon::SetValueMotionProfileHold;
						_state = 0;

						_loopTimeout = -1;
					}
					_isDone = true;
					break;
			}
		}
		instrumentation::Process(_status);
	}

	void startFilling(Segment* profile, int totalCnt) {
		CANTalon::TrajectoryPoint point;

		if(_status.hasUnderrun){
			_talon.ClearMotionProfileHasUnderrun();
		}

		_talon.ClearMotionProfileTrajectories();

		for(int i=0;i<totalCnt;++i){	// pushing the positions, velocity, time durations from Jaci's trajectory to srx motion profile trajectory
			point.position = profile[i].position / (M_PI * WHEEL_DIAMETER);	// Converting to rotations
			point.velocity = profile[i].velocity * (SECONDS_PER_MINUTE / (M_PI * WHEEL_DIAMETER));	// Converting to rotations per minute
			point.timeDurMs = (int) profile[i].dt * 1000.0;		// to milliseconds
			point.profileSlotSelect = 1;
			point.velocityOnly = false;

			point.zeroPos = false;
			if (i == 0)
				point.zeroPos = true;

			point.isLastPoint = false;
			if( (i + 1) == totalCnt )
				point.isLastPoint = true;

			_talon.PushMotionProfileTrajectory(point);
		}
	}

	void start(Segment *trajectory, int trajectoryLength) {
		_bStart = true;
		trajectoryLength_ = trajectoryLength;
		trajectory_= (Segment*)malloc(trajectoryLength_ * sizeof(Segment));
	}

	CANTalon::SetValueMotionProfile getSetValue() {
		return _setValue;
	}

	bool isDone() {
		return _isDone;
	}

private:
	Segment *trajectory_;
	int trajectoryLength_;
};

#endif /* SRC_CONTROLLERS_MOTIONPROFILEEXAMPLE_H_ */
