/*
 * RemoteControl.h
 *
 *  Created on: Jan 19, 2017
 *      Author: Lynn D
 */

#ifndef SRC_REMOTECONTROL_H_
#define SRC_REMOTECONTROL_H_

class RemoteControl {
public:
	// Joysticks
	enum Joysticks {kRightJoy, kLeftJoy};
	enum Axes {kX,kY};

	virtual void ReadControls() = 0;
	virtual double GetJoystickValue(Joysticks j, Axes a) = 0;

	// Buttons
};



#endif /* SRC_REMOTECONTROL_H_ */
