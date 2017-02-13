#ifndef BUTTONREADER_H_
#define BUTTONREADER_H_

#include "WPILib.h"

//This file outlines classes that read the states of buttons.
//ButtonReader reads the states of push buttons
class ButtonReader {
public:
	/**
	 * Sets joystick, button, gets currState from rawbutton, sets last state to currState
	 */
	ButtonReader(Joystick *joy, int buttonNum);
	virtual ~ButtonReader();
	/**
	 * Sets lastState to currState, updates currState w/button
	 */
	void ReadValue();
	/**
	 * @return current state of button (whether its currently pressed)
	 */
	bool IsDown();
	/**
	 * @return true if lastState is true and currState is false. else returns false.
	 */
	bool WasJustPressed();
	/**
	 * @return true if lastState is false and currState is true. else returns false.
	 */
	bool WasJustReleased();
	/**
	 * @return true if lastState doesn't equal currState. else return false.
	 */
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
	/**
	 * Constructs an instance of class ButtonReader and initialized the current state
	 * @param joy a Joystick
	 * @param buttonNum an int
	 */
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
	/**
	 * Initializes which joystick the switch is part of
	 * @param myJoy a Joystick
	 * @param upButton an int
	 * @param downButton an int
	 */
	SwitchReader(Joystick *myJoy, int upButton, int downButton);
	/**
	 * @return the state of the switch (up or down)
	 */
	SwitchState GetSwitchState();

private:
	Joystick *joy;
	int upB;
	int downB;
};

#endif
