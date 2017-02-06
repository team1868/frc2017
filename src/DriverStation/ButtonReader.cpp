#include <DriverStation/ButtonReader.h>
#include "WPILib.h"

//Class ButtonReader
//Constructs a joystick and sets the port of the button on the joystick, and reads the state of the button
ButtonReader::ButtonReader(Joystick* myJoystick, int myButtonNum) {
	joystick = myJoystick;
	buttonNum = myButtonNum;
	currState = joystick->GetRawButton(buttonNum);
	lastState = currState;
}

//Reads the value of the button by setting the last state to the current state and updating the current state
void ButtonReader::ReadValue() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
}

//Checks that the button was not pressed previous to the current time
bool ButtonReader::WasJustPressed() {
	return (lastState == false && currState == true);
}

//Checks that the button just changed state from pressed to released
bool ButtonReader::WasJustReleased() {
	return (lastState == true && currState == false);
}

//Checks if the state just changed either from pressed to released or released to pressed
bool ButtonReader::StateJustChanged() {
	return (lastState != currState);
}

//Checks if the button is currently pressed
bool ButtonReader::IsDown() {
	return currState;
}

//ButtonReader destructor
ButtonReader::~ButtonReader() {
}

//Class ToggleButtonReader
//Constructs an instance of class ButtonReader and initialized the current state
ToggleButtonReader::ToggleButtonReader(Joystick *joy, int buttonNum) :
	ButtonReader(joy, buttonNum) {
	currToggleState = false;
}

//Gets the current state of the toggle
bool ToggleButtonReader::GetState() {
	if (WasJustReleased()) {
		currToggleState = !currToggleState;
	}
	return (currToggleState);
}

//ToggleButtonReader destructor
ToggleButtonReader::~ToggleButtonReader() {
}

//Class SwitchReader
//Initializes which joystick the switch is part of
SwitchReader::SwitchReader(Joystick *myJoy, int upButton, int downButton) {
	joy = myJoy;
	upB = upButton;
	downB = downButton;
}

//Returns the state of the switch��up or down
SwitchState SwitchReader::GetSwitchState() {
	if (joy->GetRawButton(upB))
		return kUp;
	if (joy->GetRawButton(downB))
		return kDown;
	return kNeutral;
}
