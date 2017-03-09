#ifndef SRC_AUTO_MOTIONPROFILING_HIGHGOALAFTERRIGHTLIFT_MOTIONPROFILE_H_
#define SRC_AUTO_MOTIONPROFILING_HIGHGOALAFTERRIGHTLIFT_MOTIONPROFILE_H_

#include "Auto/MotionProfiling/MotionProfile.h"

class HighGoalAfterRightLift_MotionProfile : public MotionProfile {
public:
	HighGoalAfterRightLift_MotionProfile() {};

	virtual int GetLengthOfLeftMotionProfile() {
		return kLeftMotionProfileSz;
	}

	virtual int GetLengthOfRightMotionProfile() {
		return kRightMotionProfileSz;
	}

	virtual double (*GetLeftMotionProfile())[3] {
		return kLeftMotionProfile;
	}

	virtual double (*GetRightMotionProfile())[3] {
		return kRightMotionProfile;
	}

// Waypoint p1 = {0.0, 0.0, d2r(-60.0)};
// Waypoint p2 = {5.0, -5.0, d2r(-45.0)};

// WRONG:
//Waypoint p1 = {0.0, 0.0, 0.0};
//Waypoint p2 = {6.0, -1.0, d2r(-15)};

private:
	int kLeftMotionProfileSz = 312;

	double kLeftMotionProfile[312][3] = {
	    {1.08739e-05, 0.130487, 10},
	    {1.60409e-05, 0.0310017, 10},
	    {5.4283e-05, 0.229452, 10},
	    {0.000248934, 1.1679, 10},
	    {0.000553075, 1.82485, 10},
	    {0.000991037, 2.62778, 10},
	    {0.00158715, 3.57669, 10},
	    {0.00236575, 4.67157, 10},
	    {0.00335115, 5.91244, 10},
	    {0.0045677, 7.29926, 10},
	    {0.00603971, 8.83205, 10},
	    {0.0077915, 10.5108, 10},
	    {0.00984742, 12.3355, 10},
	    {0.0122318, 14.3061, 10},
	    {0.0149689, 16.4226, 10},
	    {0.018083, 18.685, 10},
	    {0.0215986, 21.0933, 10},
	    {0.0255398, 23.6474, 10},
	    {0.029931, 26.3473, 10},
	    {0.0347965, 29.193, 10},
	    {0.0401606, 32.1845, 10},
	    {0.0460476, 35.3217, 10},
	    {0.0524817, 38.6046, 10},
	    {0.0594872, 42.0331, 10},
	    {0.0670883, 45.6071, 10},
	    {0.0753095, 49.3267, 10},
	    {0.0841748, 53.1917, 10},
	    {0.0937084, 57.2022, 10},
	    {0.103935, 61.3579, 10},
	    {0.114878, 65.6588, 10},
	    {0.126562, 70.1049, 10},
	    {0.139011, 74.696, 10},
	    {0.15225, 79.4321, 10},
	    {0.166302, 84.313, 10},
	    {0.181192, 89.3385, 10},
	    {0.196943, 94.5086, 10},
	    {0.213581, 99.8231, 10},
	    {0.231128, 105.282, 10},
	    {0.249608, 110.885, 10},
	    {0.269047, 116.631, 10},
	    {0.289455, 122.449, 10},
	    {0.310833, 128.264, 10},
	    {0.333179, 134.077, 10},
	    {0.356493, 139.888, 10},
	    {0.380776, 145.695, 10},
	    {0.406026, 151.5, 10},
	    {0.432243, 157.302, 10},
	    {0.459426, 163.1, 10},
	    {0.487575, 168.895, 10},
	    {0.51669, 174.687, 10},
	    {0.546769, 180.474, 10},
	    {0.577812, 186.258, 10},
	    {0.609818, 192.037, 10},
	    {0.642786, 197.811, 10},
	    {0.676717, 203.581, 10},
	    {0.711608, 209.346, 10},
	    {0.747459, 215.106, 10},
	    {0.784269, 220.861, 10},
	    {0.822037, 226.609, 10},
	    {0.860762, 232.352, 10},
	    {0.900444, 238.088, 10},
	    {0.94108, 243.818, 10},
	    {0.98267, 249.54, 10},
	    {1.02521, 255.256, 10},
	    {1.06871, 260.964, 10},
	    {1.11315, 266.664, 10},
	    {1.15854, 272.356, 10},
	    {1.20488, 278.04, 10},
	    {1.25217, 283.714, 10},
	    {1.30039, 289.308, 10},
	    {1.34951, 294.747, 10},
	    {1.39952, 300.032, 10},
	    {1.45038, 305.163, 10},
	    {1.50207, 310.14, 10},
	    {1.55456, 314.962, 10},
	    {1.60783, 319.63, 10},
	    {1.66186, 324.145, 10},
	    {1.71661, 328.504, 10},
	    {1.77206, 332.71, 10},
	    {1.82819, 336.762, 10},
	    {1.88496, 340.66, 10},
	    {1.94236, 344.404, 10},
	    {2.00036, 347.994, 10},
	    {2.05893, 351.431, 10},
	    {2.11805, 354.715, 10},
	    {2.17769, 357.845, 10},
	    {2.23783, 360.823, 10},
	    {2.29844, 363.648, 10},
	    {2.35949, 366.321, 10},
	    {2.42097, 368.842, 10},
	    {2.48284, 371.212, 10},
	    {2.54507, 373.43, 10},
	    {2.60766, 375.497, 10},
	    {2.67056, 377.414, 10},
	    {2.73376, 379.181, 10},
	    {2.79722, 380.798, 10},
	    {2.86093, 382.266, 10},
	    {2.92486, 383.585, 10},
	    {2.98899, 384.756, 10},
	    {3.05329, 385.779, 10},
	    {3.11773, 386.655, 10},
	    {3.18229, 387.384, 10},
	    {3.24695, 387.967, 10},
	    {3.31169, 388.405, 10},
	    {3.37647, 388.698, 10},
	    {3.44128, 388.846, 10},
	    {3.50609, 388.85, 10},
	    {3.57087, 388.71, 10},
	    {3.63561, 388.428, 10},
	    {3.70029, 388.075, 10},
	    {3.76491, 387.719, 10},
	    {3.82947, 387.363, 10},
	    {3.89397, 387.005, 10},
	    {3.95841, 386.647, 10},
	    {4.02279, 386.287, 10},
	    {4.08711, 385.926, 10},
	    {4.15138, 385.565, 10},
	    {4.21558, 385.202, 10},
	    {4.27972, 384.839, 10},
	    {4.34379, 384.475, 10},
	    {4.40781, 384.111, 10},
	    {4.47177, 383.746, 10},
	    {4.53567, 383.38, 10},
	    {4.5995, 383.014, 10},
	    {4.66328, 382.648, 10},
	    {4.72699, 382.281, 10},
	    {4.79064, 381.914, 10},
	    {4.85423, 381.546, 10},
	    {4.91776, 381.178, 10},
	    {4.98123, 380.811, 10},
	    {5.04464, 380.442, 10},
	    {5.10799, 380.074, 10},
	    {5.17127, 379.706, 10},
	    {5.23449, 379.338, 10},
	    {5.29765, 378.969, 10},
	    {5.36075, 378.601, 10},
	    {5.42379, 378.233, 10},
	    {5.48677, 377.864, 10},
	    {5.54969, 377.496, 10},
	    {5.61254, 377.128, 10},
	    {5.67533, 376.76, 10},
	    {5.73807, 376.393, 10},
	    {5.80074, 376.025, 10},
	    {5.86335, 375.658, 10},
	    {5.9259, 375.291, 10},
	    {5.98838, 374.924, 10},
	    {6.05081, 374.557, 10},
	    {6.11317, 374.19, 10},
	    {6.17548, 373.824, 10},
	    {6.23772, 373.458, 10},
	    {6.2999, 373.093, 10},
	    {6.36202, 372.727, 10},
	    {6.42408, 372.362, 10},
	    {6.48608, 371.997, 10},
	    {6.54802, 371.633, 10},
	    {6.6099, 371.268, 10},
	    {6.67172, 370.905, 10},
	    {6.73348, 370.541, 10},
	    {6.79517, 370.178, 10},
	    {6.85681, 369.814, 10},
	    {6.91838, 369.452, 10},
	    {6.9799, 369.089, 10},
	    {7.04135, 368.727, 10},
	    {7.10275, 368.365, 10},
	    {7.16408, 368.003, 10},
	    {7.22535, 367.642, 10},
	    {7.28657, 367.281, 10},
	    {7.34772, 366.92, 10},
	    {7.40881, 366.559, 10},
	    {7.46985, 366.199, 10},
	    {7.53082, 365.839, 10},
	    {7.59173, 365.479, 10},
	    {7.65259, 365.119, 10},
	    {7.71338, 364.759, 10},
	    {7.77411, 364.4, 10},
	    {7.83479, 364.04, 10},
	    {7.8954, 363.681, 10},
	    {7.95595, 363.322, 10},
	    {8.01645, 362.963, 10},
	    {8.07688, 362.604, 10},
	    {8.13726, 362.246, 10},
	    {8.19757, 361.887, 10},
	    {8.25782, 361.529, 10},
	    {8.31802, 361.17, 10},
	    {8.37816, 360.812, 10},
	    {8.43823, 360.453, 10},
	    {8.49825, 360.095, 10},
	    {8.5582, 359.736, 10},
	    {8.6181, 359.378, 10},
	    {8.67794, 359.02, 10},
	    {8.73771, 358.661, 10},
	    {8.79743, 358.303, 10},
	    {8.85709, 357.944, 10},
	    {8.91668, 357.585, 10},
	    {8.97622, 357.226, 10},
	    {9.0357, 356.867, 10},
	    {9.09512, 356.508, 10},
	    {9.15448, 356.149, 10},
	    {9.21377, 355.79, 10},
	    {9.27301, 355.43, 10},
	    {9.33219, 355.071, 10},
	    {9.39131, 354.711, 10},
	    {9.45036, 354.319, 10},
	    {9.50933, 353.83, 10},
	    {9.5682, 353.213, 10},
	    {9.62695, 352.47, 10},
	    {9.68555, 351.599, 10},
	    {9.74398, 350.602, 10},
	    {9.80223, 349.479, 10},
	    {9.86027, 348.23, 10},
	    {9.91808, 346.857, 10},
	    {9.97564, 345.36, 10},
	    {10.0329, 343.739, 10},
	    {10.0899, 341.994, 10},
	    {10.1466, 340.127, 10},
	    {10.203, 338.137, 10},
	    {10.259, 336.026, 10},
	    {10.3146, 333.793, 10},
	    {10.3698, 331.44, 10},
	    {10.4247, 328.966, 10},
	    {10.4791, 326.372, 10},
	    {10.533, 323.659, 10},
	    {10.5865, 320.827, 10},
	    {10.6395, 317.876, 10},
	    {10.6919, 314.807, 10},
	    {10.7439, 311.62, 10},
	    {10.7953, 308.317, 10},
	    {10.8461, 304.896, 10},
	    {10.8963, 301.359, 10},
	    {10.9459, 297.706, 10},
	    {10.9949, 293.937, 10},
	    {11.0432, 290.053, 10},
	    {11.0909, 286.054, 10},
	    {11.1379, 281.941, 10},
	    {11.1842, 277.713, 10},
	    {11.2298, 273.371, 10},
	    {11.2746, 268.915, 10},
	    {11.3186, 264.346, 10},
	    {11.3619, 259.664, 10},
	    {11.4044, 254.868, 10},
	    {11.4461, 249.96, 10},
	    {11.4869, 244.94, 10},
	    {11.5268, 239.838, 10},
	    {11.566, 234.716, 10},
	    {11.6042, 229.605, 10},
	    {11.6417, 224.506, 10},
	    {11.6782, 219.416, 10},
	    {11.7139, 214.337, 10},
	    {11.7488, 209.269, 10},
	    {11.7829, 204.21, 10},
	    {11.816, 199.161, 10},
	    {11.8484, 194.121, 10},
	    {11.8799, 189.09, 10},
	    {11.9106, 184.068, 10},
	    {11.9404, 179.056, 10},
	    {11.9694, 174.051, 10},
	    {11.9976, 169.055, 10},
	    {12.025, 164.067, 10},
	    {12.0515, 159.086, 10},
	    {12.0772, 154.114, 10},
	    {12.102, 149.148, 10},
	    {12.1261, 144.19, 10},
	    {12.1493, 139.238, 10},
	    {12.1716, 134.293, 10},
	    {12.1932, 129.354, 10},
	    {12.2139, 124.422, 10},
	    {12.2339, 119.495, 10},
	    {12.253, 114.574, 10},
	    {12.2712, 109.659, 10},
	    {12.2887, 104.749, 10},
	    {12.3053, 99.8432, 10},
	    {12.3212, 94.9731, 10},
	    {12.3362, 90.1988, 10},
	    {12.3504, 85.5504, 10},
	    {12.364, 81.0273, 10},
	    {12.3767, 76.6294, 10},
	    {12.3888, 72.3563, 10},
	    {12.4002, 68.2078, 10},
	    {12.4108, 64.1834, 10},
	    {12.4209, 60.283, 10},
	    {12.4303, 56.5064, 10},
	    {12.4391, 52.8533, 10},
	    {12.4473, 49.3234, 10},
	    {12.455, 45.9167, 10},
	    {12.4621, 42.6328, 10},
	    {12.4687, 39.4716, 10},
	    {12.4748, 36.4331, 10},
	    {12.4803, 33.5169, 10},
	    {12.4855, 30.723, 10},
	    {12.4901, 28.0512, 10},
	    {12.4944, 25.5015, 10},
	    {12.4982, 23.0737, 10},
	    {12.5017, 20.7677, 10},
	    {12.5048, 18.5834, 10},
	    {12.5075, 16.5209, 10},
	    {12.51, 14.5799, 10},
	    {12.5121, 12.7605, 10},
	    {12.5139, 11.0625, 10},
	    {12.5155, 9.486, 10},
	    {12.5169, 8.03089, 10},
	    {12.518, 6.69713, 10},
	    {12.5189, 5.4847, 10},
	    {12.5196, 4.39359, 10},
	    {12.5202, 3.42377, 10},
	    {12.5206, 2.57522, 10},
	    {12.5209, 1.84793, 10},
	    {12.5211, 1.2419, 10},
	    {12.5213, 0.757113, 10},
	    {12.5213, 0.393573, 10},
	    {12.5214, 0.151275, 10},
	    {12.5214, 0.0302182, 10},
	    {12.5214, 0, 10}};

	/* ----------------------------------------------------*/

	int kRightMotionProfileSz = 312;

	double kRightMotionProfile[312][3] = {
	    {1.08739e-05, 0.130487, 10},
	    {1.60409e-05, 0.0310017, 10},
	    {3.10318e-05, 0.0899453, 10},
	    {0.000184347, 0.919893, 10},
	    {0.000423903, 1.43733, 10},
	    {0.000768864, 2.06977, 10},
	    {0.0012384, 2.81719, 10},
	    {0.00185166, 3.67961, 10},
	    {0.00262784, 4.65703, 10},
	    {0.00358608, 5.74946, 10},
	    {0.00474557, 6.95691, 10},
	    {0.00612546, 8.27938, 10},
	    {0.00774494, 9.71688, 10},
	    {0.00962318, 11.2694, 10},
	    {0.0117794, 12.9371, 10},
	    {0.0142327, 14.7198, 10},
	    {0.0170022, 16.6176, 10},
	    {0.0201073, 18.6305, 10},
	    {0.0235671, 20.7586, 10},
	    {0.0274007, 23.0019, 10},
	    {0.0316275, 25.3604, 10},
	    {0.0362665, 27.8341, 10},
	    {0.041337, 30.4232, 10},
	    {0.0468583, 33.1276, 10},
	    {0.0528495, 35.9474, 10},
	    {0.05933, 38.8827, 10},
	    {0.0663189, 41.9335, 10},
	    {0.0738355, 45.0999, 10},
	    {0.0818992, 48.3819, 10},
	    {0.0905292, 51.7797, 10},
	    {0.0997447, 55.2934, 10},
	    {0.109565, 58.9229, 10},
	    {0.12001, 62.6686, 10},
	    {0.131098, 66.5303, 10},
	    {0.14285, 70.5084, 10},
	    {0.155284, 74.6029, 10},
	    {0.168419, 78.8139, 10},
	    {0.182276, 83.1417, 10},
	    {0.196874, 87.5864, 10},
	    {0.212232, 92.1481, 10},
	    {0.22836, 96.7696, 10},
	    {0.245259, 101.393, 10},
	    {0.262929, 106.019, 10},
	    {0.28137, 110.648, 10},
	    {0.300583, 115.279, 10},
	    {0.320569, 119.914, 10},
	    {0.341328, 124.551, 10},
	    {0.362859, 129.191, 10},
	    {0.385165, 133.835, 10},
	    {0.408246, 138.483, 10},
	    {0.432102, 143.134, 10},
	    {0.456733, 147.79, 10},
	    {0.482141, 152.45, 10},
	    {0.508327, 157.114, 10},
	    {0.535291, 161.783, 10},
	    {0.563034, 166.457, 10},
	    {0.591556, 171.136, 10},
	    {0.62086, 175.821, 10},
	    {0.650945, 180.511, 10},
	    {0.681813, 185.207, 10},
	    {0.713465, 189.91, 10},
	    {0.745901, 194.62, 10},
	    {0.779124, 199.336, 10},
	    {0.813134, 204.059, 10},
	    {0.847932, 208.79, 10},
	    {0.88352, 213.529, 10},
	    {0.9199, 218.276, 10},
	    {0.957071, 223.031, 10},
	    {0.995037, 227.796, 10},
	    {1.03379, 232.511, 10},
	    {1.07331, 237.119, 10},
	    {1.11358, 241.621, 10},
	    {1.15458, 246.015, 10},
	    {1.1963, 250.303, 10},
	    {1.23871, 254.484, 10},
	    {1.28181, 258.558, 10},
	    {1.32556, 262.526, 10},
	    {1.36996, 266.387, 10},
	    {1.41498, 270.141, 10},
	    {1.46061, 273.788, 10},
	    {1.50683, 277.328, 10},
	    {1.55363, 280.76, 10},
	    {1.60097, 284.086, 10},
	    {1.64886, 287.304, 10},
	    {1.69726, 290.414, 10},
	    {1.74616, 293.416, 10},
	    {1.79555, 296.31, 10},
	    {1.8454, 299.096, 10},
	    {1.89569, 301.773, 10},
	    {1.94642, 304.341, 10},
	    {1.99755, 306.8, 10},
	    {2.04908, 309.148, 10},
	    {2.10097, 311.387, 10},
	    {2.15323, 313.516, 10},
	    {2.20581, 315.533, 10},
	    {2.25872, 317.439, 10},
	    {2.31193, 319.233, 10},
	    {2.36541, 320.915, 10},
	    {2.41916, 322.485, 10},
	    {2.47315, 323.941, 10},
	    {2.52736, 325.283, 10},
	    {2.58178, 326.511, 10},
	    {2.63639, 327.624, 10},
	    {2.69116, 328.622, 10},
	    {2.74607, 329.504, 10},
	    {2.80112, 330.27, 10},
	    {2.85627, 330.918, 10},
	    {2.91151, 331.449, 10},
	    {2.96682, 331.861, 10},
	    {3.02219, 332.215, 10},
	    {3.07762, 332.57, 10},
	    {3.13311, 332.926, 10},
	    {3.18866, 333.284, 10},
	    {3.24426, 333.643, 10},
	    {3.29993, 334.002, 10},
	    {3.35566, 334.363, 10},
	    {3.41145, 334.725, 10},
	    {3.46729, 335.087, 10},
	    {3.5232, 335.45, 10},
	    {3.57917, 335.814, 10},
	    {3.6352, 336.179, 10},
	    {3.69129, 336.544, 10},
	    {3.74744, 336.909, 10},
	    {3.80366, 337.275, 10},
	    {3.85993, 337.642, 10},
	    {3.91626, 338.009, 10},
	    {3.97266, 338.376, 10},
	    {4.02912, 338.743, 10},
	    {4.08564, 339.111, 10},
	    {4.14222, 339.479, 10},
	    {4.19886, 339.847, 10},
	    {4.25556, 340.215, 10},
	    {4.31232, 340.584, 10},
	    {4.36915, 340.952, 10},
	    {4.42603, 341.32, 10},
	    {4.48298, 341.689, 10},
	    {4.53999, 342.057, 10},
	    {4.59706, 342.425, 10},
	    {4.6542, 342.793, 10},
	    {4.71139, 343.161, 10},
	    {4.76864, 343.529, 10},
	    {4.82596, 343.897, 10},
	    {4.88334, 344.265, 10},
	    {4.94078, 344.632, 10},
	    {4.99828, 344.999, 10},
	    {5.05584, 345.366, 10},
	    {5.11346, 345.733, 10},
	    {5.17114, 346.099, 10},
	    {5.22889, 346.466, 10},
	    {5.28669, 346.831, 10},
	    {5.34456, 347.197, 10},
	    {5.40249, 347.563, 10},
	    {5.46047, 347.928, 10},
	    {5.51852, 348.292, 10},
	    {5.57663, 348.657, 10},
	    {5.6348, 349.021, 10},
	    {5.69303, 349.385, 10},
	    {5.75132, 349.749, 10},
	    {5.80968, 350.112, 10},
	    {5.86809, 350.475, 10},
	    {5.92656, 350.838, 10},
	    {5.9851, 351.201, 10},
	    {6.04369, 351.563, 10},
	    {6.10234, 351.925, 10},
	    {6.16106, 352.286, 10},
	    {6.21983, 352.648, 10},
	    {6.27867, 353.009, 10},
	    {6.33756, 353.37, 10},
	    {6.39652, 353.731, 10},
	    {6.45553, 354.091, 10},
	    {6.51461, 354.451, 10},
	    {6.57374, 354.811, 10},
	    {6.63294, 355.171, 10},
	    {6.69219, 355.531, 10},
	    {6.75151, 355.89, 10},
	    {6.81088, 356.25, 10},
	    {6.87032, 356.609, 10},
	    {6.92981, 356.968, 10},
	    {6.98937, 357.327, 10},
	    {7.04898, 357.685, 10},
	    {7.10866, 358.044, 10},
	    {7.16839, 358.403, 10},
	    {7.22818, 358.761, 10},
	    {7.28804, 359.12, 10},
	    {7.34795, 359.478, 10},
	    {7.40792, 359.837, 10},
	    {7.46795, 360.195, 10},
	    {7.52805, 360.553, 10},
	    {7.5882, 360.912, 10},
	    {7.64841, 361.27, 10},
	    {7.70868, 361.629, 10},
	    {7.76901, 361.987, 10},
	    {7.8294, 362.346, 10},
	    {7.88985, 362.705, 10},
	    {7.95036, 363.063, 10},
	    {8.01094, 363.422, 10},
	    {8.07157, 363.781, 10},
	    {8.13226, 364.141, 10},
	    {8.19301, 364.5, 10},
	    {8.25382, 364.859, 10},
	    {8.31469, 365.219, 10},
	    {8.37562, 365.579, 10},
	    {8.4366, 365.906, 10},
	    {8.49762, 366.133, 10},
	    {8.55866, 366.227, 10},
	    {8.61969, 366.188, 10},
	    {8.68069, 366.014, 10},
	    {8.74164, 365.706, 10},
	    {8.80252, 365.263, 10},
	    {8.8633, 364.684, 10},
	    {8.92396, 363.969, 10},
	    {8.98448, 363.117, 10},
	    {9.04484, 362.128, 10},
	    {9.105, 361.001, 10},
	    {9.16496, 359.737, 10},
	    {9.22468, 358.333, 10},
	    {9.28415, 356.79, 10},
	    {9.34333, 355.108, 10},
	    {9.40221, 353.286, 10},
	    {9.46077, 351.323, 10},
	    {9.51897, 349.219, 10},
	    {9.5768, 346.973, 10},
	    {9.63423, 344.585, 10},
	    {9.69124, 342.055, 10},
	    {9.7478, 339.382, 10},
	    {9.8039, 336.566, 10},
	    {9.8595, 333.606, 10},
	    {9.91458, 330.501, 10},
	    {9.96912, 327.253, 10},
	    {10.0231, 323.859, 10},
	    {10.0765, 320.32, 10},
	    {10.1293, 316.635, 10},
	    {10.1814, 312.805, 10},
	    {10.2329, 308.828, 10},
	    {10.2836, 304.704, 10},
	    {10.3337, 300.434, 10},
	    {10.3831, 296.016, 10},
	    {10.4316, 291.45, 10},
	    {10.4794, 286.737, 10},
	    {10.5264, 281.876, 10},
	    {10.5725, 276.867, 10},
	    {10.6178, 271.709, 10},
	    {10.6622, 266.437, 10},
	    {10.7058, 261.12, 10},
	    {10.7484, 255.791, 10},
	    {10.7901, 250.452, 10},
	    {10.831, 245.102, 10},
	    {10.8709, 239.742, 10},
	    {10.91, 234.372, 10},
	    {10.9482, 228.992, 10},
	    {10.9854, 223.602, 10},
	    {11.0218, 218.203, 10},
	    {11.0573, 212.795, 10},
	    {11.0918, 207.377, 10},
	    {11.1255, 201.951, 10},
	    {11.1582, 196.517, 10},
	    {11.1901, 191.074, 10},
	    {11.221, 185.623, 10},
	    {11.251, 180.165, 10},
	    {11.2802, 174.699, 10},
	    {11.3084, 169.225, 10},
	    {11.3357, 163.745, 10},
	    {11.362, 158.257, 10},
	    {11.3875, 152.763, 10},
	    {11.412, 147.263, 10},
	    {11.4357, 141.756, 10},
	    {11.4584, 136.244, 10},
	    {11.4802, 130.726, 10},
	    {11.501, 125.202, 10},
	    {11.521, 119.674, 10},
	    {11.54, 114.14, 10},
	    {11.5581, 108.637, 10},
	    {11.5753, 103.233, 10},
	    {11.5916, 97.9652, 10},
	    {11.6071, 92.8325, 10},
	    {11.6217, 87.8357, 10},
	    {11.6356, 82.9751, 10},
	    {11.6486, 78.251, 10},
	    {11.6609, 73.6635, 10},
	    {11.6724, 69.2131, 10},
	    {11.6832, 64.8999, 10},
	    {11.6934, 60.7242, 10},
	    {11.7028, 56.6862, 10},
	    {11.7116, 52.7861, 10},
	    {11.7198, 49.024, 10},
	    {11.7274, 45.4002, 10},
	    {11.7343, 41.9149, 10},
	    {11.7408, 38.568, 10},
	    {11.7467, 35.3599, 10},
	    {11.752, 32.2907, 10},
	    {11.7569, 29.3603, 10},
	    {11.7614, 26.569, 10},
	    {11.7653, 23.9169, 10},
	    {11.7689, 21.404, 10},
	    {11.7721, 19.0304, 10},
	    {11.7749, 16.7962, 10},
	    {11.7773, 14.7014, 10},
	    {11.7795, 12.7461, 10},
	    {11.7813, 10.9303, 10},
	    {11.7828, 9.25413, 10},
	    {11.7841, 7.71756, 10},
	    {11.7852, 6.32063, 10},
	    {11.786, 5.06337, 10},
	    {11.7867, 3.9458, 10},
	    {11.7872, 2.96792, 10},
	    {11.7875, 2.12975, 10},
	    {11.7878, 1.43131, 10},
	    {11.7879, 0.872592, 10},
	    {11.788, 0.453605, 10},
	    {11.788, 0.174349, 10},
	    {11.788, 0.0348275, 10},
	    {11.788, 0, 10}};



};

#endif /* SRC_AUTO_MOTIONPROFILING_HIGHGOALAFTERRIGHTLIFT__MOTIONPROFILE_H_ */
