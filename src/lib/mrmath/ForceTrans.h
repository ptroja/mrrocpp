#ifndef __FORCETRANS_H
#define __FORCETRANS_H

#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {

#define X_AXIS_ARM 0
#define Y_AXIS_ARM 0
#define Z_AXIS_ARM -0.032
#define Z_AXIS_GRAVITY_FORCE -242.0

class ForceTrans
{

protected:
	bool initialized;
	double tool_weight;
	//	lib::K_vector gravity_force_in_base;
	lib::Ft_vector gravity_force_torque_in_base;
	lib::Ft_vector reaction_force_torque_in_sensor;
	lib::K_vector gravity_arm_in_wrist;

	//	lib::K_vector reaction_force_in_sensor;
	//	lib::K_vector reaction_torque_in_sensor;
	//	lib::Homog_matrix initialisation_frame;

	lib::Homog_matrix sensor_frame;

	//		lib::Homog_matrix sensor_frame_translation;
	//		lib::Homog_matrix sensor_frame_rotation;

	lib::Ft_tr ft_tool_mass_center_translation;

	lib::Ft_tr ft_tr_sensor_in_wrist;

	//	lib::Ft_v_tr ft_tr_sensor_translation_matrix;
	//	lib::Ft_v_tr ft_tr_inv_sensor_translation_matrix;
	//	lib::Ft_v_tr ft_tr_sensor_rotation_matrix;
	//	lib::Ft_v_tr ft_tr_inv_sensor_rotation_matrix;

public:
	//	ForceTrans(const lib::Homog_matrix & init_frame, const lib::Homog_matrix & s_frame);										// standardowy tool
	ForceTrans(const lib::Homog_matrix & init_frame, const lib::Homog_matrix & s_frame, const double weight, const lib::K_vector & point_of_gravity);
	void defineTool(const lib::Homog_matrix & init_frame, const double weight, const lib::K_vector & point_of_gravity);
	void synchro(const lib::Homog_matrix & init_frame);
	lib::Ft_vector getForce(const lib::Ft_vector inputForceTorque, const lib::Homog_matrix curr_frame);
};

} // namespace lib
} // namespace mrrocpp


#endif
