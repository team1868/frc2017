#include <ButtonReader.h>

ButtonReader::ButtonReader(Joystick *myJoystick, int myButtonNum) {
	joystick = myJoystick;
	buttonNum = myButtonNum;
	currButtonState = myJoystick->GetRawButton(myButtonNum);
	lastButtonState = currButtonState;
}

void ButtonReader::ReadValue() {
	lastButtonState = currButtonState;
	currButtonState = joystick->GetRawButton(buttonNum);
}

bool ButtonReader::IsDown() {
	return currButtonState;
}

bool ButtonReader::WasJustPressed() {
	return (lastButtonState == false && currButtonState == true);
}

bool ButtonReader::WasJustReleased() {
	return (lastButtonState == true && currButtonState == false);
}

bool ButtonReader::StateJustChanged() {
	return (lastButtonState != currButtonState);
}
