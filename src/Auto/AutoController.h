#ifndef SRC_AUTO_AUTOCONTROLLER_H_
#define SRC_AUTO_AUTOCONTROLLER_H_

#include "Auto/Modes/AutoMode.h"

class AutoController {
public:
	/**
	 * Constructor AutoController that does not set automode
	 */
	AutoController();
	/**
	 * Constructor AutoController that allows automode to be set
	 * @param autoMode an AutoMode
	 */
	AutoController(AutoMode *autoMode);
	virtual ~AutoController() {}
	/**
	 * SetAutomousMode is a mutator that sets AutoMode
	 * @param autoMode is an AutoMode
	 */
	void SetAutonomousMode(AutoMode *autoMode);
	void StartAutonomous();		// TODO
	/**
	 * create a queue for automode and initializes it
	 */
	void Init();
	/**
	 * updates automode
	 * @param currTimeSec a double that is the current time in seconds
	 * @param deltaTimeSec a double that is the change in time
	 */
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();	// TODO
	/**
	 * @return true when AutoMode isDone
	 */
	bool IsDone();

private:
	AutoMode *autoMode;
};

#endif /* SRC_AUTO_AUTOCONTROLLER_H_ */
