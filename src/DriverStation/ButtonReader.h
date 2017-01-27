#ifndef BUTTONREADER_H_
#define BUTTONREADER_H_

#include "WPILib.h"

//This file outlines classes that read the states of buttons.
//ButtonReader reads the states of push buttons
class ButtonReader {
public:
	ButtonReader(Joystick *joy, int buttonNum);
	virtual ~ButtonReader();
	void ReadValue();
	bool IsDown();
	bool WasJustPressed();
	bool WasJustReleased();
	bool StateJustChanged();

private:
	Joystick *joystick;
	int buttonNum;
	bool lastState;
	bool currState;
};

//ToggleButtonReader reads the states of toggles
class ToggleButtonReader : public ButtonReader {
public:
	ToggleButtonReader(Joystick *joy, int buttonNum);
	virtual ~ToggleButtonReader();
	virtual bool GetState();

private:
	bool currToggleState;
};

enum SwitchState {
	kUp = 1,
	kNeutral = 0,
	kDown = -1,
};

//SwitchReaader reads the state of switches
class SwitchReader {
public:
	SwitchReader(Joystick *myJoy, int upButton, int downButton);
	SwitchState GetSwitchState();

private:
	Joystick *joy;
	int upB;
	int downB;
};

#endif
