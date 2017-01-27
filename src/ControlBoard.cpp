/*
 * ControlBoard.cpp
 *
 *  Created on: Jan 19, 2017
 *      Author: Lynn D
 */

#include <ControlBoard.h>

ControlBoard::ControlBoard() {
	leftJoy_ = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy_ = new Joystick(RIGHT_JOY_USB_PORT);

	leftJoyX_ = 0;
	leftJoyY_ = 0;
	rightJoyX_ = 0;
	rightJoyY_ = 0;
}

void ControlBoard::ReadControls() {
	leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = leftJoy_ ->GetY();
	rightJoyX_ = rightJoy_ ->GetX();
	rightJoyY_ = rightJoy_ ->GetY();
}

double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
		case(kRightJoy):
			switch(a) {
				case(kX):
					return rightJoyX_;
				case(kY):
					return rightJoyY_;
			}
			break;
		case(kLeftJoy):
			switch(a) {
				case(kX):
					return leftJoyX_;
				case(kY):
					return leftJoyY_;
			}
			break;
	}
	return 0.0;
}

void ControlBoard::ReadAllButtons() {

}

ControlBoard::~ControlBoard() {
	// TODO Auto-generated destructor stub
}

