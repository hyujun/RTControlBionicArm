#include "PropertyDefinition.h"

#define ENC_OFFSET_J1 4369066
#define ENC_OFFSET_J2 8956586
#define ENC_OFFSET_J3 2184533
#define ENC_OFFSET_J4 0
#define ENC_OFFSET_J5 54272

#define HARMONIC_100 100
#define GEAR_ELBOW 26*2
#define GEAR_PS 53


#define ABS_ENC_19 524288
#define ABS_ENC_18 262144
#define INC_ENC_10 1024*4

#define MAX_CURRENT_TYPE1 1.8
#define MAX_CURRENT_TYPE2 2
#define MAX_CURRENT_TYPE3 3.21
#define MAX_CURRENT_TYPE4 3.34

#define TORQUE_CONST_TYPE1 0.183 	// Nm/A Arm 1,2,3
#define TORQUE_CONST_TYPE2 0.091 	// Nm/A Arm 4
#define TORQUE_CONST_TYPE3 0.0369 	// Nm/A Arm 5,6
#define TORQUE_CONST_TYPE4 0.014 	// Nm/A Waist 1,2

// {w_x, w_y, w_z, q_x, q_y, q_z, l_x, l_y, l_z}
robot_kinematic_info serial_Kinematic_info[] = {
		{0, 0, 1,
		0.0, -200.0e-3, 660.0e-3,
		0.0, -354.7e-3, 660.0e-3},			// 1
		{0, -1, 0,
		0.0, -354.7e-3, 660.0e-3,
		0.0, -354.7e-3, 514.2e-3},		// 2
		{0, 0, -1,
		0.0, -354.7e-3, 514.2e-3,
		0.0, -215.0e-3, 299.8e-3},		// 3
		{0, 1, 0,
		0.0, -215.0e-3, 299.8e-3,
		0.0, -325.0e-3, 800.0e-3},		// 4
		{1, 0, 0,
		0.0, -325.0e-3, 800.0e-3,
		0.0, -325.0e-3, 558.0e-3},		// 5
		{0, 0, 1,
		0.0, -325.0e-3, 558.0e-3,
		-24.5e-3, -325.0e-3, 400.0e-3},		// 6
		{0, 1, 0,
		-24.5e-3, -325.0e-3, 400.0e-3,
		0.0, -325.0e-3, 245.0e-3},		// 7
};

//{MASS,
// J_Ixx, J_Iyy, J_Izz, J_Ixy, J_Iyz, J_Izx,
// CoM_x, CoM_y, CoM_z},
robot_dynamic_info serial_Dynamic_info[] = {
		{11.91,
				83854563.43e-9, 70969371.34e-9, 53062013.96e-9, -135841.54e-9, -15432807.98e-9, -625787.35e-9,
				-0.1e-3, -3.05e-3, 449.99e-3}, 				// 1
		{9.901,
				0.1408, 0.05, 0.108, 0.0, 0.003643, 0.0007,
				1.0139e-3, 3.6731e-3, 643.6171e-3},			// 2
		{3.35,
				9155194.92e-9, 7656654.39e-9, 6023745.1e-9, 163898.7e-9, -981692.47e-9, -52182.13e-9,
				-0.7e-3, -142.89e-3, 781.38e-3},			// 3
		{3.51,
				11833182.94e-9, 5963801.91e-9, 12674379.1e-9, 245028.38e-9, -81204.78e-9, -121068.48e-9,
				-0.21e-3, -293.04e-3, 800.77e-3},			// 4
		{2.59,
				14349158.07e-9, 15957517.14e-9, 3811118.16e-9, 0.0, -1126404.95e-9, 0.0,
				-2.54e-3, -325.18e-3, 649.61e-3},			// 5
		{2.49,
				12305788.09e-9, 10949889.37e-9, 3675298.10e-9, 0.0, -322614.65e-9, 1326295.35e-9,
				-19.09e-3, -322.41e-3, 449.31e-3},			// 6

};

// {HarmonicRatio, EncoderResolution, MaximumContinuousCurrent, TorqueConstant, AbsolutePositionOffset}
robot_motor_info serial_Motor_info[] = {
		{HARMONIC_100, ABS_ENC_18, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J1},
		{HARMONIC_100, ABS_ENC_18, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J2},
		{HARMONIC_100, ABS_ENC_18, MAX_CURRENT_TYPE2, TORQUE_CONST_TYPE2, ENC_OFFSET_J3},
		{GEAR_ELBOW, INC_ENC_10, MAX_CURRENT_TYPE3, TORQUE_CONST_TYPE3, ENC_OFFSET_J4},
		{GEAR_PS, INC_ENC_10, MAX_CURRENT_TYPE4, TORQUE_CONST_TYPE4, ENC_OFFSET_J5},
};

FrictionMap frictionmap[] ={
		{0.05, 18.5},	//1
		{0.05, 10.5},	//2
		{0.05, 6.8},	//3
		{0.05, 7.0},	//4
		{0.05, 6.5},	//5
		{0.05, 4.0},	//6
};

FrictionTanh frictiontanh[] = {
		{215, 24.83, 22.54, 20.5, 11.81, 0.715}, 			// 1
		{215, 24.83, 22.54, 34.7, 11.81, 0.715},	 		// 2
		{41.87, 573.8, 592.2, 6.811, 12.96, 0.138}, 		// 3
		{19.99, 35.09, 64.18, 17.6, 36.34, 2.398e-7},		// 4
		{2.069, 59.79, 95.02, 18.5, 3.299, 2.49e-7},		// 5
		{151.3, 16.42, 17.26, 4.56, 14.47, 0.05998}, 		// 6
};



homing_info hominginfo[] = {
		{3276800, 	-1, ABS_ENC_18*3, 900}, //offset (int32_t)round((45.0/360.0)*ABS_ENC_18*HARMONIC_100)
		{8228409, 	-1, ABS_ENC_18*3, 900}, //offset (int32_t)round((113.0/360.0)*ABS_ENC_18*HARMONIC_100)
		{-2184533, 	-2, ABS_ENC_18*3, 900}, //offset -(int32_t)round((30.0/360.0)*ABS_ENC_18*HARMONIC_100)
		{0, 		-1, INC_ENC_10*2, 900},
		{-54272,	-2, INC_ENC_10*3, 500}, //offset (int32_t)round((90.0/360.0)*INC_ENC_10*GEAR_PS)
};
