#ifndef SRC_BUTTONREADER_H_
#define SRC_BUTTONREADER_H_

#include "WPILib.h"

class ButtonReader {
public:
	ButtonReader(Joystick *joystick, int buttonNum);
	virtual ~ButtonReader() {}

	void ReadValue();
	bool IsDown();
	bool WasJustPressed();
	bool WasJustReleased();
	bool StateJustChanged();

private:
	Joystick *joystick;
	int buttonNum;
	bool lastButtonState;
	bool currButtonState;
};

#endif /* SRC_BUTTONREADER_H_ */
