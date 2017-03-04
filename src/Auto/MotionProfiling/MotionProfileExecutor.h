#ifndef MotionProfileExample__h_
#define MotionProfileExample__h_
/**
 * Example logic for firing and managing motion profiles.
 * This example sends MPs, waits for them to finish
 * Although this code uses a CANTalon, nowhere in this module do we changeMode() or call set() to change the output.
 * This is done in Robot.java to demonstrate how to change control modes on the fly.
 *
 * The only routines we call on Talon are....
 *
 * changeMotionControlFramePeriod
 *
 * getMotionProfileStatus
 * clearMotionProfileHasUnderrun     to get status and potentially clear the error flag.
 *
 * pushMotionProfileTrajectory
 * clearMotionProfileTrajectories
 * processMotionProfileBuffer,   to push/clear, and process the trajectory points.
 *
 * getControlMode, to check if we are in Motion Profile Control mode.
 *
 * Example of advanced features not demonstrated here...
 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position.
 */
#include "Auto/MotionProfiling/Instrumentation.h"
#include "Auto/MotionProfiling/LiftOne_MotionProfile.h"
#include "WPILib.h"
#include "CANTalon.h"

class MotionProfileExecutor {

public:
	/**
	 * The status of the motion profile executer and buffer inside the Talon.
	 * Instead of creating a new one every time we call getMotionProfileStatus,
	 * keep one copy.
	 */
	CANTalon::MotionProfileStatus status_;
	/**
	 * reference to the talon we plan on manipulating. We will not changeMode()
	 * or call set(), just get motion profile status and make decisions based on
	 * motion profile.
	 */
	CANTalon & talon_;
	/**
	 * State machine to make sure we let enough of the motion profile stream to
	 * talon before we fire it.
	 */
	int state_ = 0;
	/**
	 * Any time you have a state machine that waits for external events, its a
	 * good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
	 * down to '0' which will print an error message. Counting loops is not a
	 * very accurate method of tracking timeout, but this is just conservative
	 * timeout. Getting time-stamps would certainly work too, this is just
	 * simple (no need to worry about timer overflows).
	 */
	int loopTimeout_ = -1;
	/**
	 * If start() gets called, this flag is set and in the control() we will
	 * service it.
	 */
	bool bStart_ = false;

	bool isLeft_ = false;
	bool hasStarted_ = false;		// to change

	bool isDone_ = false;

	double **profile_;
	int trajLength_;

	/**
	 * Since the CANTalon.set() routine is mode specific, deduce what we want
	 * the set value to be and let the calling module apply it whenever we
	 * decide to switch to MP mode.
	 */
	CANTalon::SetValueMotionProfile setValue_ = CANTalon::SetValueMotionProfileDisable;
	/**
	 * How many trajectory points do we wait for before firing the motion
	 * profile.
	 */
	static const int kMinPointsInTalon = 5;
	/**
	 * Just a state timeout to make sure we don't get stuck anywhere. Each loop
	 * is about 20ms.
	 */
	static const int kNumLoopsTimeout = 10;
	/**
	 * Lets create a periodic task to funnel our trajectory points into our
	 * talon. It doesn't need to be very accurate, just needs to keep pace with
	 * the motion profiler executer. Now if you're trajectory points are slow,
	 * there is no need to do this, just call
	 * _talon.processMotionProfileBuffer() in your teleop loop. Generally
	 * speaking you want to call it at least twice as fast as the duration of
	 * your trajectory points. So if they are firing every 20ms, you should call
	 * every 10ms.
	 */
	void PeriodicTask()
	{
		/* keep Talons happy by moving the points from top-buffer to bottom-buffer */
		talon_.ProcessMotionProfileBuffer();
	}
	/**
	 * Lets create a periodic task to funnel our trajectory points into our talon.
	 * It doesn't need to be very accurate, just needs to keep pace with the motion
	 * profiler executer.
	 */
	Notifier notifer_;

	MotionProfileExecutor(CANTalon & talon, double profile[][3], int trajLength) : talon_(talon), notifer_(&MotionProfileExecutor::PeriodicTask, this)
	{
		/*
		 * since our MP is 10ms per point, set the control frame rate and the
		 * notifer to half that
		 */
		talon_.ChangeMotionControlFramePeriod(5);

		/* start our tasking */
		notifer_.StartPeriodic(0.005);

		trajLength_ = trajLength;

		profile_ = (double**)malloc(trajLength_ * sizeof(double*));
		for (int i = 0; i < trajLength_; i++) {
			profile_[i] = (double*)malloc(3 * sizeof(double));
		}

		for (int i = 0; i < trajLength_; i++) {
			memcpy(profile_[i], profile[i], sizeof(double) * 3);
		}
	}

	/**
	 * Called to clear Motion profile buffer and reset state info during
	 * disabled and when Talon is not in MP control mode.
	 */
	void reset() {
		/*
		 * Let's clear the buffer just in case user decided to disable in the
		 * middle of an MP, and now we have the second half of a profile just
		 * sitting in memory.
		 */
		talon_.ClearMotionProfileTrajectories();
		talon_.ClearError();
		talon_.ClearIaccum();
		talon_.ClearStickyFaults();
		/* When we do re-enter motionProfile control mode, stay disabled. */
		setValue_ = CANTalon::SetValueMotionProfileDisable;
		/* When we do start running our state machine start at the beginning. */
		state_ = 0;
		loopTimeout_ = -1;
		/*
		 * If application wanted to start an MP before, ignore and wait for next
		 * button press
		 */
		bStart_ = false;
	}

	/**
	 * Called every loop.
	 */
	void control() {
		/* Get the motion profile status every loop */
		talon_.GetMotionProfileStatus(status_);

		/*
		 * track time, this is rudimentary but that's okay, we just want to make
		 * sure things never get stuck.
		 */
		if (loopTimeout_ < 0) {
			/* do nothing, timeout is disabled */
		} else {
			/* our timeout is nonzero */
			if (loopTimeout_ == 0) {
				/*
				 * something is wrong. Talon is not present, unplugged, breaker
				 * tripped
				 */
				instrumentation::OnNoProgress();
			} else {
				--loopTimeout_;
			}
		}

		/* first check if we are in MP mode */
		if (talon_.GetControlMode() != CANSpeedController::kMotionProfile){
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			state_ = 0;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp
			 * progress, and possibly interrupting MPs if thats what you want to
			 * do.
			 */
			switch (state_) {
				case 0: /* wait for application to tell us to start an MP */

					if (bStart_) {
						bStart_ = false;

						setValue_ = CANTalon::SetValueMotionProfileDisable;
						startFilling();
						/*
						 * MP is` being sent to CAN bus, wait a small amount of time
						 */
						state_ = 1;
						loopTimeout_ = kNumLoopsTimeout;

						talon_.ClearIaccum();
						talon_.ClearError();
					}
					break;
				case 1: /*
						 * wait for MP to stream to Talon, really just the first few
						 * points
						 */
					/* do we have a minimum numberof points in Talon */
					if (status_.btmBufferCnt > kMinPointsInTalon) {
						//SmartDashboard::PutString("HELLO CONTROL", "BUFFER > MINPOINTS");
						/* start (once) the motion profile */
						setValue_ = CANTalon::SetValueMotionProfileEnable;
						/* MP will start once the control frame gets scheduled */
						state_ = 2;
						loopTimeout_ = kNumLoopsTimeout;
					}
					break;
				case 2: /* check the status of the MP */
					/*
					 * if talon is reporting things are good, keep adding to our
					 * timeout. Really this is so that you can unplug your talon in
					 * the middle of an MP and react to it.
					 */
					if (status_.isUnderrun == false) {
						loopTimeout_ = kNumLoopsTimeout;
					}
					/*
					 * If we are executing an MP and the MP finished, start loading
					 * another. We will go into hold state so robot servo's
					 * position.
					 */
					if (status_.activePointValid && status_.activePoint.isLastPoint) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						setValue_ = CANTalon::SetValueMotionProfileHold;
//						state_ = 0;
						state_ = 3;
						loopTimeout_ = -1;

						for (int i = 0; i < trajLength_; i++) {
							free(profile_[i]);
						}
						free(profile_);
						printf("IN LAST POINT MOTION PROFILE\n");
					}
					break;
				case 3:
					SmartDashboard::PutNumber("HI CONTROL", 3);
					if (setValue_ == CANTalon::SetValueMotionProfileHold) {
						setValue_ = CANTalon::SetValueMotionProfileDisable;
//						talon_.SetControlMode(CANTalon::kPercentVbus);
//						talon_.Set( 0 );
//						talon_.Reset();
//						talon_.ClearIaccum();
						/* clear our buffer and put everything into a known state */
						printf("IN CASE 3 MOTION PROFILE\n");
						isDone_ = true;		// DONE!!
					}
					break;
			}
		}
		/* printfs and/or logging */
		instrumentation::Process(status_);
	}

	/** Start filling the MPs to all of the involved Talons. */
	void startFilling()
	{
		startFilling(profile_, trajLength_);
	}

	void startFilling(double **profile, int totalCnt)
	{
		SmartDashboard::PutString("Filling", "hi");
		/* create an empty point */
		CANTalon::TrajectoryPoint point;

		/* did we get an underrun condition since last time we checked ? */
		if(status_.hasUnderrun){
			/* better log it so we know about it */
			instrumentation::OnUnderrun();
			/*
			 * clear the error. This is what seperates "has underrun" from
			 * "is underrun", because the former is cleared by the application.
			 * That way, we never miss logging it.
			 */
			talon_.ClearMotionProfileHasUnderrun();
		}

		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		talon_.ClearMotionProfileTrajectories();

		/* This is fast since it's just into our TOP buffer */
		for(int i=0;i<totalCnt;++i){
			/* for each point, fill our structure and pass it to API */
			point.position = profile[i][0]; /*
															 * copy the position, velocity, and
															 * time from current row
															 */
			point.velocity = profile[i][1];

			point.timeDurMs = (int) profile[i][2];

			SmartDashboard::PutNumber("MP Position", point.position);
			SmartDashboard::PutNumber("MP Velocity", point.velocity);
			SmartDashboard::PutNumber("MP Time Dur MS", point.timeDurMs);
			point.profileSlotSelect = 0; /*
											 * which set of gains would you like to
											 * use?
											 */
			point.velocityOnly = false; /*
										 * set true to not do any position
										 * servo, just velocity feedforward
										 */

			point.zeroPos = false;
			if (i == 0)
				point.zeroPos = true; /* set this to true on the first point */

			point.isLastPoint = false;
			if( (i + 1) == totalCnt )
				point.isLastPoint = true; /*
											 * set this to true on the last point
											 */

			talon_.PushMotionProfileTrajectory(point);
		}
	}

	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	void start() {
		hasStarted_ = true;
		bStart_ = true;
	}

	/**
	 *
	 * @return the output value to pass to Talon's set() routine. 0 for disable
	 *         motion-profile output, 1 for enable motion-profile, 2 for hold
	 *         current motion profile trajectory point.
	 */
	CANTalon::SetValueMotionProfile getSetValue() {
		return setValue_;
	}
};
#endif // MotionProfileExample__h_
