#ifndef SRC_AUTO_MOTIONPROFILING_LIFTONE_MOTIONPROFILE_H_
#define SRC_AUTO_MOTIONPROFILING_LIFTONE_MOTIONPROFILE_H_

#include "Auto/MotionProfiling/MotionProfile.h"

class LiftOne_MotionProfile : public MotionProfile {
public:
	LiftOne_MotionProfile() {};

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

private:

	int kLeftMotionProfileSz = 214;

	double kLeftMotionProfile[214][3] = {		// TO CHANGE
		{1.57064e-05, 0.188476, 10},
		{5.28949e-05, 0.223131, 10},
		{0.00024329, 1.14237, 10},
		{0.000581789, 2.031, 10},
		{0.00111074, 3.17372, 10},
		{0.00187254, 4.57079, 10},
		{0.00290963, 6.22253, 10},
		{0.00426454, 8.12946, 10},
		{0.0059799, 10.2922, 10},
		{0.0080985, 12.7116, 10},
		{0.0106633, 15.3887, 10},
		{0.0137174, 18.3248, 10},
		{0.0173043, 21.5214, 10},
		{0.0214677, 24.9803, 10},
		{0.0262517, 28.7038, 10},
		{0.0317007, 32.6944, 10},
		{0.0378386, 36.8269, 10},
		{0.0446679, 40.9758, 10},
		{0.0521917, 45.1431, 10},
		{0.0604135, 49.3308, 10},
		{0.069337, 53.541, 10},
		{0.0789663, 57.7757, 10},
		{0.0893058, 62.0372, 10},
		{0.10036, 66.3277, 10},
		{0.112135, 70.6494, 10},
		{0.124636, 75.0048, 10},
		{0.137869, 79.3962, 10},
		{0.15184, 83.8261, 10},
		{0.166556, 88.2971, 10},
		{0.182025, 92.8117, 10},
		{0.198253, 97.3726, 10},
		{0.21525, 101.982, 10},
		{0.233024, 106.644, 10},
		{0.251584, 111.36, 10},
		{0.27094, 116.133, 10},
		{0.291101, 120.965, 10},
		{0.312077, 125.86, 10},
		{0.333881, 130.82, 10},
		{0.356522, 135.847, 10},
		{0.380012, 140.943, 10},
		{0.404364, 146.111, 10},
		{0.42959, 151.352, 10},
		{0.455701, 156.666, 10},
		{0.48271, 162.055, 10},
		{0.51063, 167.519, 10},
		{0.539472, 173.057, 10},
		{0.56925, 178.666, 10},
		{0.599974, 184.346, 10},
		{0.631656, 190.09, 10},
		{0.664305, 195.895, 10},
		{0.697931, 201.753, 10},
		{0.73254, 207.656, 10},
		{0.768139, 213.592, 10},
		{0.804731, 219.55, 10},
		{0.842316, 225.513, 10},
		{0.880894, 231.465, 10},
		{0.920458, 237.385, 10},
		{0.961, 243.252, 10},
		{1.00251, 249.041, 10},
		{1.04496, 254.726, 10},
		{1.08834, 260.278, 10},
		{1.13262, 265.669, 10},
		{1.17776, 270.868, 10},
		{1.22374, 275.846, 10},
		{1.2705, 280.573, 10},
		{1.318, 285.025, 10},
		{1.3662, 289.174, 10},
		{1.41503, 293.002, 10},
		{1.46445, 296.491, 10},
		{1.51439, 299.63, 10},
		{1.56479, 302.412, 10},
		{1.6156, 304.838, 10},
		{1.66675, 306.91, 10},
		{1.71819, 308.641, 10},
		{1.76986, 310.044, 10},
		{1.82172, 311.139, 10},
		{1.87371, 311.949, 10},
		{1.92579, 312.499, 10},
		{1.97793, 312.815, 10},
		{2.03008, 312.927, 10},
		{2.08223, 312.862, 10},
		{2.13433, 312.646, 10},
		{2.18638, 312.307, 10},
		{2.23836, 311.868, 10},
		{2.29025, 311.351, 10},
		{2.34205, 310.776, 10},
		{2.39374, 310.16, 10},
		{2.44533, 309.518, 10},
		{2.49681, 308.861, 10},
		{2.54817, 308.2, 10},
		{2.59943, 307.542, 10},
		{2.65056, 306.777, 10},
		{2.70153, 305.803, 10},
		{2.7523, 304.634, 10},
		{2.80285, 303.278, 10},
		{2.85314, 301.745, 10},
		{2.90314, 300.038, 10},
		{2.95284, 298.163, 10},
		{3.00219, 296.121, 10},
		{3.05118, 293.912, 10},
		{3.09977, 291.537, 10},
		{3.14793, 288.993, 10},
		{3.19564, 286.278, 10},
		{3.24288, 283.388, 10},
		{3.2896, 280.32, 10},
		{3.33577, 277.07, 10},
		{3.38137, 273.592, 10},
		{3.42635, 269.883, 10},
		{3.47068, 265.982, 10},
		{3.51433, 261.884, 10},
		{3.55726, 257.585, 10},
		{3.59944, 253.082, 10},
		{3.64084, 248.373, 10},
		{3.68141, 243.454, 10},
		{3.72113, 238.325, 10},
		{3.75996, 232.985, 10},
		{3.79787, 227.436, 10},
		{3.83482, 221.68, 10},
		{3.87077, 215.722, 10},
		{3.9057, 209.569, 10},
		{3.93957, 203.229, 10},
		{3.97236, 196.714, 10},
		{4.00403, 190.068, 10},
		{4.0346, 183.376, 10},
		{4.06404, 176.685, 10},
		{4.09238, 170.002, 10},
		{4.1196, 163.339, 10},
		{4.14572, 156.708, 10},
		{4.17074, 150.125, 10},
		{4.19467, 143.609, 10},
		{4.21754, 137.18, 10},
		{4.23935, 130.861, 10},
		{4.26013, 124.678, 10},
		{4.2799, 118.656, 10},
		{4.29871, 112.824, 10},
		{4.31658, 107.21, 10},
		{4.33355, 101.841, 10},
		{4.34967, 96.7429, 10},
		{4.365, 91.9411, 10},
		{4.37957, 87.4563, 10},
		{4.39346, 83.306, 10},
		{4.40671, 79.5031, 10},
		{4.41938, 76.0556, 10},
		{4.43154, 72.9656, 10},
		{4.44325, 70.2301, 10},
		{4.45456, 67.8403, 10},
		{4.46552, 65.7823, 10},
		{4.47619, 64.0376, 10},
		{4.48662, 62.5837, 10},
		{4.49686, 61.395, 10},
		{4.50693, 60.4437, 10},
		{4.51688, 59.7004, 10},
		{4.52674, 59.1354, 10},
		{4.53652, 58.7188, 10},
		{4.54626, 58.4221, 10},
		{4.55596, 58.2177, 10},
		{4.56564, 58.0801, 10},
		{4.57531, 57.9859, 10},
		{4.58496, 57.914, 10},
		{4.5946, 57.8455, 10},
		{4.60423, 57.7642, 10},
		{4.61384, 57.6559, 10},
		{4.62342, 57.5088, 10},
		{4.63297, 57.3131, 10},
		{4.64248, 57.0609, 10},
		{4.65194, 56.746, 10},
		{4.66134, 56.3637, 10},
		{4.67065, 55.9105, 10},
		{4.67988, 55.3843, 10},
		{4.68902, 54.7838, 10},
		{4.69803, 54.1086, 10},
		{4.70693, 53.3589, 10},
		{4.71568, 52.5357, 10},
		{4.72429, 51.6404, 10},
		{4.73273, 50.6745, 10},
		{4.74101, 49.6402, 10},
		{4.7491, 48.5398, 10},
		{4.75699, 47.3756, 10},
		{4.76469, 46.1502, 10},
		{4.77216, 44.8662, 10},
		{4.77942, 43.5264, 10},
		{4.78644, 42.1334, 10},
		{4.79322, 40.69, 10},
		{4.79976, 39.1989, 10},
		{4.80603, 37.6626, 10},
		{4.81205, 36.0839, 10},
		{4.81779, 34.4652, 10},
		{4.82326, 32.8091, 10},
		{4.82844, 31.1181, 10},
		{4.83334, 29.3945, 10},
		{4.83795, 27.6407, 10},
		{4.84226, 25.8589, 10},
		{4.84627, 24.0513, 10},
		{4.84997, 22.2201, 10},
		{4.85337, 20.3673, 10},
		{4.85645, 18.4951, 10},
		{4.85922, 16.6053, 10},
		{4.86167, 14.7273, 10},
		{4.86383, 12.9241, 10},
		{4.8657, 11.232, 10},
		{4.86731, 9.65332, 10},
		{4.86867, 8.19004, 10},
		{4.86981, 6.84389, 10},
		{4.87075, 5.61631, 10},
		{4.8715, 4.50845, 10},
		{4.87209, 3.52128, 10},
		{4.87253, 2.65553, 10},
		{4.87285, 1.91179, 10},
		{4.87306, 1.29048, 10},
		{4.87319, 0.791894, 10},
		{4.87326, 0.416223, 10},
		{4.87329, 0.163577, 10},
		{4.8733, 0.0340105, 10},
		{4.8733, 0, 10}};

	/* -------------------------------------*/

	int kRightMotionProfileSz = 214;

	double kRightMotionProfile[214][3] = {
			{1.57064e-05, 0.188476, 10},
			{5.28949e-05, 0.223131, 10},
			{0.00024329, 1.14237, 10},
			{0.000581789, 2.031, 10},
			{0.00111074, 3.17372, 10},
			{0.00187254, 4.57079, 10},
			{0.00290963, 6.22253, 10},
			{0.00426454, 8.12946, 10},
			{0.0059799, 10.2922, 10},
			{0.0080985, 12.7116, 10},
			{0.0106633, 15.3887, 10},
			{0.0137174, 18.3248, 10},
			{0.0173043, 21.5214, 10},
			{0.0214677, 24.9803, 10},
			{0.0262517, 28.7038, 10},
			{0.0317007, 32.6944, 10},
			{0.0378386, 36.8269, 10},
			{0.0446679, 40.9758, 10},
			{0.0521917, 45.1431, 10},
			{0.0604135, 49.3308, 10},
			{0.069337, 53.541, 10},
			{0.0789663, 57.7757, 10},
			{0.0893058, 62.0372, 10},
			{0.10036, 66.3277, 10},
			{0.112135, 70.6494, 10},
			{0.124636, 75.0048, 10},
			{0.137869, 79.3962, 10},
			{0.15184, 83.8261, 10},
			{0.166556, 88.2971, 10},
			{0.182025, 92.8117, 10},
			{0.198253, 97.3726, 10},
			{0.21525, 101.982, 10},
			{0.233024, 106.644, 10},
			{0.251584, 111.36, 10},
			{0.27094, 116.133, 10},
			{0.291101, 120.965, 10},
			{0.312077, 125.86, 10},
			{0.333881, 130.82, 10},
			{0.356522, 135.847, 10},
			{0.380012, 140.943, 10},
			{0.404364, 146.111, 10},
			{0.42959, 151.352, 10},
			{0.455701, 156.666, 10},
			{0.48271, 162.055, 10},
			{0.51063, 167.519, 10},
			{0.539472, 173.057, 10},
			{0.56925, 178.666, 10},
			{0.599974, 184.346, 10},
			{0.631656, 190.09, 10},
			{0.664305, 195.895, 10},
			{0.697931, 201.753, 10},
			{0.73254, 207.656, 10},
			{0.768139, 213.592, 10},
			{0.804731, 219.55, 10},
			{0.842316, 225.513, 10},
			{0.880894, 231.465, 10},
			{0.920458, 237.385, 10},
			{0.961, 243.252, 10},
			{1.00251, 249.041, 10},
			{1.04496, 254.726, 10},
			{1.08834, 260.278, 10},
			{1.13262, 265.669, 10},
			{1.17776, 270.868, 10},
			{1.22374, 275.846, 10},
			{1.2705, 280.573, 10},
			{1.318, 285.025, 10},
			{1.3662, 289.174, 10},
			{1.41503, 293.002, 10},
			{1.46445, 296.491, 10},
			{1.51439, 299.63, 10},
			{1.56479, 302.412, 10},
			{1.6156, 304.838, 10},
			{1.66675, 306.91, 10},
			{1.71819, 308.641, 10},
			{1.76986, 310.044, 10},
			{1.82172, 311.139, 10},
			{1.87371, 311.949, 10},
			{1.92579, 312.499, 10},
			{1.97793, 312.815, 10},
			{2.03008, 312.927, 10},
			{2.08223, 312.862, 10},
			{2.13433, 312.646, 10},
			{2.18638, 312.307, 10},
			{2.23836, 311.868, 10},
			{2.29025, 311.351, 10},
			{2.34205, 310.776, 10},
			{2.39374, 310.16, 10},
			{2.44533, 309.518, 10},
			{2.49681, 308.861, 10},
			{2.54817, 308.2, 10},
			{2.59943, 307.542, 10},
			{2.65056, 306.777, 10},
			{2.70153, 305.803, 10},
			{2.7523, 304.634, 10},
			{2.80285, 303.278, 10},
			{2.85314, 301.745, 10},
			{2.90314, 300.038, 10},
			{2.95284, 298.163, 10},
			{3.00219, 296.121, 10},
			{3.05118, 293.912, 10},
			{3.09977, 291.537, 10},
			{3.14793, 288.993, 10},
			{3.19564, 286.278, 10},
			{3.24288, 283.388, 10},
			{3.2896, 280.32, 10},
			{3.33577, 277.07, 10},
			{3.38137, 273.592, 10},
			{3.42635, 269.883, 10},
			{3.47068, 265.982, 10},
			{3.51433, 261.884, 10},
			{3.55726, 257.585, 10},
			{3.59944, 253.082, 10},
			{3.64084, 248.373, 10},
			{3.68141, 243.454, 10},
			{3.72113, 238.325, 10},
			{3.75996, 232.985, 10},
			{3.79787, 227.436, 10},
			{3.83482, 221.68, 10},
			{3.87077, 215.722, 10},
			{3.9057, 209.569, 10},
			{3.93957, 203.229, 10},
			{3.97236, 196.714, 10},
			{4.00403, 190.068, 10},
			{4.0346, 183.376, 10},
			{4.06404, 176.685, 10},
			{4.09238, 170.002, 10},
			{4.1196, 163.339, 10},
			{4.14572, 156.708, 10},
			{4.17074, 150.125, 10},
			{4.19467, 143.609, 10},
			{4.21754, 137.18, 10},
			{4.23935, 130.861, 10},
			{4.26013, 124.678, 10},
			{4.2799, 118.656, 10},
			{4.29871, 112.824, 10},
			{4.31658, 107.21, 10},
			{4.33355, 101.841, 10},
			{4.34967, 96.7429, 10},
			{4.365, 91.9411, 10},
			{4.37957, 87.4563, 10},
			{4.39346, 83.306, 10},
			{4.40671, 79.5031, 10},
			{4.41938, 76.0556, 10},
			{4.43154, 72.9656, 10},
			{4.44325, 70.2301, 10},
			{4.45456, 67.8403, 10},
			{4.46552, 65.7823, 10},
			{4.47619, 64.0376, 10},
			{4.48662, 62.5837, 10},
			{4.49686, 61.395, 10},
			{4.50693, 60.4437, 10},
			{4.51688, 59.7004, 10},
			{4.52674, 59.1354, 10},
			{4.53652, 58.7188, 10},
			{4.54626, 58.4221, 10},
			{4.55596, 58.2177, 10},
			{4.56564, 58.0801, 10},
			{4.57531, 57.9859, 10},
			{4.58496, 57.914, 10},
			{4.5946, 57.8455, 10},
			{4.60423, 57.7642, 10},
			{4.61384, 57.6559, 10},
			{4.62342, 57.5088, 10},
			{4.63297, 57.3131, 10},
			{4.64248, 57.0609, 10},
			{4.65194, 56.746, 10},
			{4.66134, 56.3637, 10},
			{4.67065, 55.9105, 10},
			{4.67988, 55.3843, 10},
			{4.68902, 54.7838, 10},
			{4.69803, 54.1086, 10},
			{4.70693, 53.3589, 10},
			{4.71568, 52.5357, 10},
			{4.72429, 51.6404, 10},
			{4.73273, 50.6745, 10},
			{4.74101, 49.6402, 10},
			{4.7491, 48.5398, 10},
			{4.75699, 47.3756, 10},
			{4.76469, 46.1502, 10},
			{4.77216, 44.8662, 10},
			{4.77942, 43.5264, 10},
			{4.78644, 42.1334, 10},
			{4.79322, 40.69, 10},
			{4.79976, 39.1989, 10},
			{4.80603, 37.6626, 10},
			{4.81205, 36.0839, 10},
			{4.81779, 34.4652, 10},
			{4.82326, 32.8091, 10},
			{4.82844, 31.1181, 10},
			{4.83334, 29.3945, 10},
			{4.83795, 27.6407, 10},
			{4.84226, 25.8589, 10},
			{4.84627, 24.0513, 10},
			{4.84997, 22.2201, 10},
			{4.85337, 20.3673, 10},
			{4.85645, 18.4951, 10},
			{4.85922, 16.6053, 10},
			{4.86167, 14.7273, 10},
			{4.86383, 12.9241, 10},
			{4.8657, 11.232, 10},
			{4.86731, 9.65332, 10},
			{4.86867, 8.19004, 10},
			{4.86981, 6.84389, 10},
			{4.87075, 5.61631, 10},
			{4.8715, 4.50845, 10},
			{4.87209, 3.52128, 10},
			{4.87253, 2.65553, 10},
			{4.87285, 1.91179, 10},
			{4.87306, 1.29048, 10},
			{4.87319, 0.791894, 10},
			{4.87326, 0.416223, 10},
			{4.87329, 0.163577, 10},
			{4.8733, 0.0340105, 10},
			{4.8733, 0, 10}};
};


#endif /* SRC_AUTO_MOTIONPROFILING_LIFTONE_MOTIONPROFILE_H_ */
