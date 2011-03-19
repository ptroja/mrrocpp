#if !defined(_POLYCRANK_KIN_MODEL)
#define _POLYCRANK_KIN_MODEL

// Definicja klasy kinematic_model.
//#include "base/kinematics/kinematic_model_with_tool.h"
//#include "base/kinematics/kinematic_model.h"
//#include "robot/hi_moxa/hi_moxa.h"
//#include "robot/polycrank/edp_e_polycrank.h"

#include "base/kinematics/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace polycrank {

class model : public common::kinematic_model
{
protected:
	//! Synchronization position.
	double synchro_motor_position;
	//! Motor increments to external position (in meters) ratio.
	double motor_to_intext_ratio;
	//! Method responsible for kinematic parameters setting.
	virtual void set_kinematic_parameters(void);
	//Checks whether given motor increments are valid.
	//otor_position Motor position to be validated.
	virtual void check_motor_position(const lib::MotorArray & motor_position) const;
	//Checks whether given internal coordinates are valid.
	virtual void check_joints(const lib::JointArray & q) const;

public:
	//! Constructor.
	model(void);

	//Computes internal coordinates for given the motor increments (position) values.
	//[in] local_current_motor_pos Motor increments.
	//[out] local_current_joints Computed joints.
	virtual void mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints);

	//Computes motor increments from internal coordinates.
	//[out] local_desired_motor_pos_new Computed motor increment.
	//[in] local_desired_joints Current joints settings.
	virtual void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints);

};


} // namespace polycrank
} // namespace kinematic
} // namespace mrrocpp


#endif

