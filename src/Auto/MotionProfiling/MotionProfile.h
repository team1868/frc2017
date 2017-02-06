/*
 * MotionProfile.h
 *
 *  Created on: Feb 5, 2017
 *      Author: maggiewang
 */

#ifndef SRC_AUTO_MOTIONPROFILING_MOTIONPROFILE_H_
#define SRC_AUTO_MOTIONPROFILING_MOTIONPROFILE_H_

class MotionProfile {
public:
	int kLeftMotionProfileSz = 0;
	double kLeftMotionProfile[][3];

	int kRightMotionProfileSz = 0;
	double kRightMotionProfile[][3];

	virtual double GetLengthOfLeftMotionProfile() {
		return kLeftMotionProfileSz;
	}

	virtual double GetLengthOfRightMotionProfile() {
		return kRightMotionProfileSz;
	}

	virtual double (*GetLeftMotionProfile())[3] {
	    return kLeftMotionProfile;
	}

	virtual double (*GetRightMotionProfile())[3] {
	    return kRightMotionProfile;
	}

	virtual ~MotionProfile() {}
};

#endif /* SRC_AUTO_MOTIONPROFILING_MOTIONPROFILE_H_ */
