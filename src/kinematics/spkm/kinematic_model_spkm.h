/*
 * \file kinematic_model_spkm.h
 * \brief File containing declaration of the kinematic_model_spkm class.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */

#ifndef KINEMATIC_MODEL_SPKM_H_
#define KINEMATIC_MODEL_SPKM_H_

#include "kinematics/common/kinematic_model.h"
#include "kinematics/spkm/kinematic_parameters_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

/*!
 * \class kinematic_model_spkm
 * \brief Class solving the problem of inverse kinematics for Exechon parallel kinematc machine with spherical wrist attached to its upper platform.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */
class kinematic_model_spkm: public common::kinematic_model
{
	protected:
		//! Kinematic parameters of both: pkm and spherical wrist attached to it.
		kinematic_parameters_spkm params;

		//! Sets parameters used by given kinematics.
		void set_kinematic_parameters(void);

		//! Checks whether given motor increments are valid.
		void check_motor_position(const double motor_position[]);

		//! Checks whether given internal coordinates are valid.
		void check_joints(const double q[]);

	public:
		//! Constructor.
		kinematic_model_spkm(void);

		//! Computes internal coordinates basing on the motor increments (position).
		void mp2i_transform(const double* local_current_motor_pos, double* local_current_joints);

		//! Computes motor increments from internal coordinates.
		void i2mp_transform(double* local_desired_motor_pos_new, double* local_desired_joints);

		//! Solves direct kinematics - EMPTY!
		void direct_kinematics_transform(const double* local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

		//! Solves inverse kinematics.
		void inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, lib::Homog_matrix& local_desired_end_effector_frame);

};

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


#endif

