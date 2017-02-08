#include <Auto/AutoController.h>

AutoController::AutoController() {
	autoMode = nullptr;
}

AutoController::AutoController(AutoMode *myAutoMode){
	autoMode = myAutoMode;
}

void AutoController::SetAutonomousMode(AutoMode *myAutoMode) {
	autoMode = myAutoMode;
}

void AutoController::Init() {
	autoMode->CreateQueue();
	autoMode->Init();
}

void AutoController::Update(double currTimeSec, double deltaTimeSec) {
	autoMode->Update(currTimeSec, deltaTimeSec);
}

void AutoController::Reset() {

}

bool AutoController::IsDone() {
	return autoMode->IsDone();
}
