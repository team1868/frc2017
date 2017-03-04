int kLeftMotionProfileSz = 212;

double kLeftMotionProfile[212][3] = {
    {-2.71572e-05, -0.325886, 10},
    {-3.66039e-05, -0.0566804, 10},
    {-0.000113867, -0.46358, 10},
    {-0.000251267, -0.824399, 10},
    {-0.000466064, -1.28878, 10},
    {-0.000775606, -1.85725, 10},
    {-0.00119737, -2.5306, 10},
    {-0.00174903, -3.30994, 10},
    {-0.00244848, -4.19672, 10},
    {-0.00331396, -5.19288, 10},
    {-0.00436409, -6.3008, 10},
    {-0.005618, -7.52345, 10},
    {-0.00709541, -8.86445, 10},
    {-0.00881677, -10.3281, 10},
    {-0.0108034, -11.9196, 10},
    {-0.0130775, -13.645, 10},
    {-0.0156538, -15.4574, 10},
    {-0.0185386, -17.309, 10},
    {-0.0217394, -19.2048, 10},
    {-0.0252644, -21.1502, 10},
    {-0.0291229, -23.1506, 10},
    {-0.0333249, -25.2118, 10},
    {-0.0378815, -27.3397, 10},
    {-0.0428048, -29.5403, 10},
    {-0.0481081, -31.8198, 10},
    {-0.0538056, -34.1847, 10},
    {-0.0599125, -36.6413, 10},
    {-0.0664452, -39.1963, 10},
    {-0.0734212, -41.856, 10},
    {-0.080859, -44.627, 10},
    {-0.0887783, -47.5157, 10},
    {-0.0971997, -50.5284, 10},
    {-0.106145, -53.671, 10},
    {-0.115636, -56.9494, 10},
    {-0.125698, -60.369, 10},
    {-0.136354, -63.9348, 10},
    {-0.147629, -67.6515, 10},
    {-0.15955, -71.5232, 10},
    {-0.172142, -75.5534, 10},
    {-0.185433, -79.745, 10},
    {-0.199449, -84.1005, 10},
    {-0.21422, -88.6214, 10},
    {-0.229771, -93.3087, 10},
    {-0.246131, -98.1628, 10},
    {-0.263329, -103.183, 10},
    {-0.28139, -108.369, 10},
    {-0.300343, -113.718, 10},
    {-0.320215, -119.229, 10},
    {-0.341031, -124.898, 10},
    {-0.362818, -130.722, 10},
    {-0.385601, -136.697, 10},
    {-0.409404, -142.819, 10},
    {-0.434251, -149.083, 10},
    {-0.460165, -155.485, 10},
    {-0.487168, -162.018, 10},
    {-0.515281, -168.679, 10},
    {-0.544525, -175.462, 10},
    {-0.574919, -182.361, 10},
    {-0.606481, -189.372, 10},
    {-0.639229, -196.489, 10},
    {-0.67318, -203.707, 10},
    {-0.70835, -211.023, 10},
    {-0.744756, -218.431, 10},
    {-0.78241, -225.928, 10},
    {-0.821329, -233.511, 10},
    {-0.861525, -241.175, 10},
    {-0.903011, -248.92, 10},
    {-0.945802, -256.743, 10},
    {-0.989909, -264.643, 10},
    {-1.03535, -272.618, 10},
    {-1.08212, -280.668, 10},
    {-1.13026, -288.794, 10},
    {-1.17976, -296.998, 10},
    {-1.23064, -305.28, 10},
    {-1.28291, -313.644, 10},
    {-1.33659, -322.092, 10},
    {-1.3917, -330.63, 10},
    {-1.44824, -339.262, 10},
    {-1.50624, -347.994, 10},
    {-1.56571, -356.833, 10},
    {-1.62668, -365.787, 10},
    {-1.68915, -374.866, 10},
    {-1.75317, -384.08, 10},
    {-1.81874, -393.441, 10},
    {-1.8859, -402.961, 10},
    {-1.95468, -412.657, 10},
    {-2.0251, -422.544, 10},
    {-2.09721, -432.642, 10},
    {-2.17104, -442.969, 10},
    {-2.24663, -453.551, 10},
    {-2.324, -464.235, 10},
    {-2.40314, -474.864, 10},
    {-2.48405, -485.451, 10},
    {-2.56672, -496.011, 10},
    {-2.65115, -506.559, 10},
    {-2.73733, -517.113, 10},
    {-2.82528, -527.691, 10},
    {-2.915, -538.31, 10},
    {-3.0065, -548.99, 10},
    {-3.09979, -559.747, 10},
    {-3.19489, -570.599, 10},
    {-3.29182, -581.559, 10},
    {-3.39059, -592.636, 10},
    {-3.49123, -603.837, 10},
    {-3.59375, -615.157, 10},
    {-3.69817, -626.493, 10},
    {-3.80447, -637.814, 10},
    {-3.91267, -649.167, 10},
    {-4.02275, -660.494, 10},
    {-4.1347, -671.714, 10},
    {-4.24849, -682.724, 10},
    {-4.36405, -693.393, 10},
    {-4.48131, -703.562, 10},
    {-4.60016, -713.048, 10},
    {-4.72043, -721.64, 10},
    {-4.84195, -729.111, 10},
    {-4.96448, -735.222, 10},
    {-5.08777, -739.736, 10},
    {-5.21151, -742.429, 10},
    {-5.33536, -743.105, 10},
    {-5.45896, -741.614, 10},
    {-5.58196, -737.975, 10},
    {-5.70403, -732.438, 10},
    {-5.8249, -725.2, 10},
    {-5.94429, -716.346, 10},
    {-6.06196, -706.004, 10},
    {-6.17768, -694.337, 10},
    {-6.29127, -681.532, 10},
    {-6.40257, -667.79, 10},
    {-6.51145, -653.313, 10},
    {-6.61784, -638.298, 10},
    {-6.72166, -622.925, 10},
    {-6.82288, -607.357, 10},
    {-6.92151, -591.738, 10},
    {-7.01754, -576.185, 10},
    {-7.111, -560.797, 10},
    {-7.20195, -545.65, 10},
    {-7.29041, -530.803, 10},
    {-7.37646, -516.299, 10},
    {-7.46016, -502.167, 10},
    {-7.54156, -488.422, 10},
    {-7.62074, -475.075, 10},
    {-7.69776, -462.125, 10},
    {-7.77269, -449.567, 10},
    {-7.84559, -437.393, 10},
    {-7.91652, -425.59, 10},
    {-7.98554, -414.143, 10},
    {-8.05272, -403.037, 10},
    {-8.11809, -392.254, 10},
    {-8.18172, -381.779, 10},
    {-8.24365, -371.594, 10},
    {-8.30393, -361.681, 10},
    {-8.3626, -352.026, 10},
    {-8.41971, -342.611, 10},
    {-8.47528, -333.423, 10},
    {-8.52935, -324.446, 10},
    {-8.58196, -315.668, 10},
    {-8.63314, -307.074, 10},
    {-8.68292, -298.655, 10},
    {-8.73132, -290.397, 10},
    {-8.77837, -282.291, 10},
    {-8.82409, -274.326, 10},
    {-8.8685, -266.493, 10},
    {-8.91163, -258.785, 10},
    {-8.9535, -251.192, 10},
    {-8.99412, -243.707, 10},
    {-9.0335, -236.323, 10},
    {-9.07168, -229.034, 10},
    {-9.10865, -221.833, 10},
    {-9.14443, -214.715, 10},
    {-9.17905, -207.674, 10},
    {-9.2125, -200.706, 10},
    {-9.2448, -193.805, 10},
    {-9.27596, -186.969, 10},
    {-9.30599, -180.191, 10},
    {-9.3349, -173.469, 10},
    {-9.3627, -166.799, 10},
    {-9.3894, -160.178, 10},
    {-9.415, -153.602, 10},
    {-9.43951, -147.068, 10},
    {-9.46294, -140.575, 10},
    {-9.48529, -134.118, 10},
    {-9.50657, -127.696, 10},
    {-9.52679, -121.306, 10},
    {-9.54595, -114.946, 10},
    {-9.56405, -108.614, 10},
    {-9.5811, -102.308, 10},
    {-9.59711, -96.0256, 10},
    {-9.61207, -89.7653, 10},
    {-9.62599, -83.5254, 10},
    {-9.63887, -77.304, 10},
    {-9.65072, -71.0997, 10},
    {-9.66154, -64.9108, 10},
    {-9.67133, -58.7358, 10},
    {-9.68009, -52.5733, 10},
    {-9.68784, -46.5018, 10},
    {-9.69463, -40.7108, 10},
    {-9.70051, -35.3099, 10},
    {-9.70556, -30.2976, 10},
    {-9.70984, -25.6723, 10},
    {-9.71341, -21.4328, 10},
    {-9.71634, -17.5782, 10},
    {-9.7187, -14.1076, 10},
    {-9.72053, -11.0203, 10},
    {-9.72192, -8.31576, 10},
    {-9.72292, -5.99364, 10},
    {-9.72359, -4.05362, 10},
    {-9.72401, -2.49547, 10},
    {-9.72423, -1.31905, 10},
    {-9.72432, -0.524297, 10},
    {-9.72433, -0.111165, 10},
    {-9.72433, -0, 10}};

/* ----------------------------------------------------*/

int kRightMotionProfileSz = 212;

double kRightMotionProfile[212][3] = {
    {-2.71572e-05, -0.325886, 10},
    {-0.0001663, -0.834859, 10},
    {-0.000577866, -2.46939, 10},
    {-0.0013095, -4.38978, 10},
    {-0.00245256, -6.85837, 10},
    {-0.00409833, -9.87465, 10},
    {-0.00633797, -13.4378, 10},
    {-0.00926243, -17.5468, 10},
    {-0.0129624, -22.2, 10},
    {-0.0175284, -27.3957, 10},
    {-0.0230503, -33.1314, 10},
    {-0.0296176, -39.4041, 10},
    {-0.0373194, -46.2103, 10},
    {-0.0462436, -53.5455, 10},
    {-0.0564777, -61.4047, 10},
    {-0.068108, -69.7818, 10},
    {-0.0811743, -78.3977, 10},
    {-0.0956701, -86.9745, 10},
    {-0.111588, -95.507, 10},
    {-0.12892, -103.99, 10},
    {-0.147656, -112.418, 10},
    {-0.167787, -120.785, 10},
    {-0.189301, -129.085, 10},
    {-0.212186, -137.313, 10},
    {-0.23643, -145.462, 10},
    {-0.262018, -153.525, 10},
    {-0.288934, -161.497, 10},
    {-0.317162, -169.37, 10},
    {-0.346685, -177.139, 10},
    {-0.377485, -184.796, 10},
    {-0.409541, -192.336, 10},
    {-0.442832, -199.751, 10},
    {-0.477339, -207.037, 10},
    {-0.513036, -214.187, 10},
    {-0.549902, -221.196, 10},
    {-0.587912, -228.058, 10},
    {-0.62704, -234.769, 10},
    {-0.667261, -241.326, 10},
    {-0.708549, -247.724, 10},
    {-0.750875, -253.961, 10},
    {-0.794214, -260.034, 10},
    {-0.838538, -265.941, 10},
    {-0.883818, -271.682, 10},
    {-0.930027, -277.256, 10},
    {-0.977138, -282.664, 10},
    {-1.02512, -287.907, 10},
    {-1.07395, -292.986, 10},
    {-1.1236, -297.903, 10},
    {-1.17405, -302.663, 10},
    {-1.22526, -307.267, 10},
    {-1.27721, -311.72, 10},
    {-1.32988, -316.027, 10},
    {-1.38325, -320.191, 10},
    {-1.43728, -324.218, 10},
    {-1.49197, -328.112, 10},
    {-1.54728, -331.88, 10},
    {-1.6032, -335.525, 10},
    {-1.65971, -339.055, 10},
    {-1.71679, -342.472, 10},
    {-1.77442, -345.784, 10},
    {-1.83259, -348.994, 10},
    {-1.89127, -352.107, 10},
    {-1.95046, -355.127, 10},
    {-2.01014, -358.058, 10},
    {-2.07029, -360.904, 10},
    {-2.1309, -363.668, 10},
    {-2.19196, -366.351, 10},
    {-2.25345, -368.957, 10},
    {-2.31536, -371.486, 10},
    {-2.37769, -373.939, 10},
    {-2.44041, -376.317, 10},
    {-2.50351, -378.619, 10},
    {-2.56698, -380.844, 10},
    {-2.63082, -382.991, 10},
    {-2.69499, -385.055, 10},
    {-2.7595, -387.035, 10},
    {-2.82432, -388.926, 10},
    {-2.88944, -390.723, 10},
    {-2.95484, -392.419, 10},
    {-3.02051, -394.008, 10},
    {-3.08642, -395.482, 10},
    {-3.15256, -396.832, 10},
    {-3.2189, -398.046, 10},
    {-3.28542, -399.114, 10},
    {-3.35209, -400.022, 10},
    {-3.41889, -400.754, 10},
    {-3.48577, -401.296, 10},
    {-3.55271, -401.627, 10},
    {-3.61966, -401.727, 10},
    {-3.68659, -401.574, 10},
    {-3.75342, -400.992, 10},
    {-3.82006, -399.813, 10},
    {-3.8864, -398.025, 10},
    {-3.95233, -395.612, 10},
    {-4.01776, -392.559, 10},
    {-4.08256, -388.848, 10},
    {-4.14664, -384.462, 10},
    {-4.20987, -379.382, 10},
    {-4.27214, -373.59, 10},
    {-4.33331, -367.068, 10},
    {-4.39328, -359.8, 10},
    {-4.45191, -351.772, 10},
    {-4.50907, -342.974, 10},
    {-4.56464, -333.401, 10},
    {-4.61848, -323.057, 10},
    {-4.67047, -311.909, 10},
    {-4.72047, -299.988, 10},
    {-4.76836, -287.383, 10},
    {-4.81405, -274.152, 10},
    {-4.85745, -260.376, 10},
    {-4.89848, -246.158, 10},
    {-4.93708, -231.63, 10},
    {-4.97324, -216.949, 10},
    {-5.00696, -202.3, 10},
    {-5.03827, -187.893, 10},
    {-5.06726, -173.955, 10},
    {-5.09405, -160.726, 10},
    {-5.11879, -148.442, 10},
    {-5.14168, -137.328, 10},
    {-5.16294, -127.579, 10},
    {-5.18283, -119.348, 10},
    {-5.20163, -112.749, 10},
    {-5.2196, -107.858, 10},
    {-5.23705, -104.668, 10},
    {-5.25423, -103.095, 10},
    {-5.2714, -103.011, 10},
    {-5.28877, -104.253, 10},
    {-5.30655, -106.632, 10},
    {-5.32487, -109.948, 10},
    {-5.34387, -113.999, 10},
    {-5.36363, -118.589, 10},
    {-5.38422, -123.537, 10},
    {-5.40567, -128.678, 10},
    {-5.42798, -133.872, 10},
    {-5.45115, -138.999, 10},
    {-5.47514, -143.961, 10},
    {-5.49992, -148.681, 10},
    {-5.52544, -153.101, 10},
    {-5.55164, -157.178, 10},
    {-5.57845, -160.884, 10},
    {-5.60582, -164.201, 10},
    {-5.63367, -167.121, 10},
    {-5.66194, -169.644, 10},
    {-5.69057, -171.774, 10},
    {-5.71949, -173.521, 10},
    {-5.74864, -174.897, 10},
    {-5.77796, -175.916, 10},
    {-5.80739, -176.594, 10},
    {-5.83689, -176.948, 10},
    {-5.86639, -176.996, 10},
    {-5.89584, -176.753, 10},
    {-5.92522, -176.238, 10},
    {-5.95446, -175.465, 10},
    {-5.98354, -174.451, 10},
    {-6.0124, -173.212, 10},
    {-6.04103, -171.76, 10},
    {-6.06938, -170.111, 10},
    {-6.09743, -168.276, 10},
    {-6.12514, -166.267, 10},
    {-6.15249, -164.097, 10},
    {-6.17945, -161.775, 10},
    {-6.206, -159.311, 10},
    {-6.23212, -156.716, 10},
    {-6.25779, -153.996, 10},
    {-6.28298, -151.161, 10},
    {-6.30769, -148.217, 10},
    {-6.33188, -145.173, 10},
    {-6.35555, -142.034, 10},
    {-6.37869, -138.806, 10},
    {-6.40127, -135.496, 10},
    {-6.42329, -132.109, 10},
    {-6.44473, -128.648, 10},
    {-6.46558, -125.121, 10},
    {-6.48584, -121.529, 10},
    {-6.50549, -117.878, 10},
    {-6.52451, -114.172, 10},
    {-6.54292, -110.414, 10},
    {-6.56068, -106.607, 10},
    {-6.57781, -102.754, 10},
    {-6.59429, -98.8593, 10},
    {-6.61011, -94.9247, 10},
    {-6.62527, -90.953, 10},
    {-6.63976, -86.9467, 10},
    {-6.65358, -82.9082, 10},
    {-6.66671, -78.8398, 10},
    {-6.67917, -74.7435, 10},
    {-6.69094, -70.6213, 10},
    {-6.70202, -66.4753, 10},
    {-6.71241, -62.3072, 10},
    {-6.72209, -58.1188, 10},
    {-6.73108, -53.9118, 10},
    {-6.73936, -49.6878, 10},
    {-6.74693, -45.4483, 10},
    {-6.7538, -41.195, 10},
    {-6.75995, -36.9291, 10},
    {-6.76541, -32.7083, 10},
    {-6.77018, -28.6689, 10},
    {-6.77433, -24.8911, 10},
    {-6.7779, -21.3766, 10},
    {-6.78092, -18.1268, 10},
    {-6.78344, -15.1429, 10},
    {-6.78551, -12.426, 10},
    {-6.78717, -9.97681, 10},
    {-6.78847, -7.79609, 10},
    {-6.78945, -5.88435, 10},
    {-6.79016, -4.24199, 10},
    {-6.79064, -2.86932, 10},
    {-6.79093, -1.76655, 10},
    {-6.79109, -0.933806, 10},
    {-6.79115, -0.37118, 10},
    {-6.79116, -0.0787005, 10},
    {-6.79116, -0, 10}};

