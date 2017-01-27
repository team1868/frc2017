/*
 * ControlBoard.h
 *
 *  Created on: Jan 19, 2017
 *      Author: Lynn D
 */
#ifndef SRC_CONTROLBOARD1_H_
#define SRC_CONTROLBOARD1_H_

#include "ButtonReader.h"
#include "WPILib.h"
#include "RemoteControl.h"
#include "RobotPorts2017.h"

class ControlBoard : public RemoteControl {
public:
	ControlBoard();

	void ReadControls();
	double GetJoystickValue(Joysticks j, Axes a);
	virtual ~ControlBoard();

private:
	// Desired values for driving and pivoting
	double leftJoyX_, leftJoyY_, rightJoyX_, rightJoyY_;

	// Joysticks
	Joystick *leftJoy_, *rightJoy_;

	void ReadAllButtons();
};

#endif /* SRC_CONTROLBOARD1_H_ */
