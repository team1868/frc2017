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
#include <pathfinder/pathfinder.h>


double const pi = 3.1415926;

class MotionProfileExample {
public:
	CANTalon::MotionProfileStatus _status;
	CANTalon & _talon;
	int _state = 0;
	int _loopTimeout = -1;
	bool _bStart = false;
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

		if(_talon.GetControlMode() != CANSpeedController::kMotionProfile){
			_state = 0;
		} else {
			switch (_state) {
				case 0:
					if (_bStart) {
						_bStart = false;

						_setValue = CANTalon::SetValueMotionProfileDisable;
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
					break;
			}
		}
	}

	void startFilling(){
		//startFilling(kMotionProfile, kMotionProfileSz);
		//startFilling() // TODO
	}

	void startFilling(Segment* profile, int totalCnt) {
		CANTalon::TrajectoryPoint point;

		if(_status.hasUnderrun){
			_talon.ClearMotionProfileHasUnderrun();
		}

		_talon.ClearMotionProfileTrajectories();

		for(int i=0;i<totalCnt;++i){	// pushing the positions, velocity, time durations from Jaci's trajectory to srx motion profile trajectory
			// Assuming wheels are 7.5 inches and converted to meters which is .1905 m
			// Assuming position and velocity are in meters for testing purposes
			point.position = profile[i].position / (.1905 * pi);	// from trajectory
			point.velocity = profile[i].velocity / (.1905 * pi);		// from trajectory
			point.timeDurMs = (int) profile[i].dt;		// from trajectory
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

	void start(Segment* trajectory, int trajectoryLength) {
		_bStart = true;
		startFilling(trajectory, trajectoryLength);	// moved startFIlling() from control() to transfer the Segment array and length into startFilling()
	}

	CANTalon::SetValueMotionProfile getSetValue() {
		return _setValue;
	}
};

#endif /* SRC_CONTROLLERS_MOTIONPROFILEEXAMPLE_H_ */
