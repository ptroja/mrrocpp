#ifndef __FORCETRANS_H
#define __FORCETRANS_H

#include "lib/mathtr.h"

#define X_AXIS_ARM 0
#define Y_AXIS_ARM 0
#define Z_AXIS_ARM -0.032
#define Z_AXIS_GRAVITY_FORCE -242.0

class ForceTrans
{

protected:

const short force_sensor_name;

	bool initialized;
	double tool_weight;
	//	K_vector gravity_force_in_base;
	Ft_v_vector gravity_force_torque_in_base;
	Ft_v_vector reaction_force_torque_in_sensor;
	K_vector gravity_arm_in_wrist;
	
	//	K_vector reaction_force_in_sensor;
	//	K_vector reaction_torque_in_sensor;
	//	Homog_matrix initialisation_frame;
		Homog_matrix sensor_frame;
//		Homog_matrix sensor_frame_translation;
//		Homog_matrix sensor_frame_rotation;
	
	Ft_v_tr ft_tool_mass_center_translation;
	
	Ft_v_tr ft_tr_sensor_in_wrist;

	
//	Ft_v_tr ft_tr_sensor_translation_matrix;
//	Ft_v_tr ft_tr_inv_sensor_translation_matrix;
//	Ft_v_tr ft_tr_sensor_rotation_matrix;
//	Ft_v_tr ft_tr_inv_sensor_rotation_matrix;
	
public:
	//	ForceTrans(const Homog_matrix & init_frame, const Homog_matrix & s_frame);										// standardowy tool
	ForceTrans(const short l_force_sensor_name, const Homog_matrix & init_frame, const Homog_matrix & s_frame	, 
		const double weight, const K_vector & point_of_gravity);
	void defineTool(const Homog_matrix & init_frame, const double weight, const K_vector & point_of_gravity);
	void synchro(const Homog_matrix & init_frame);
	double* getForce(const double[6], const Homog_matrix & curr_frame);
};

#endif
