/*!
 * \file kinematic_model_with_local_corrector.h
 * \brief File containing the declaration of kinematic_model_with_local_corrector class.
 *
 * \author tkornuta
 * \date Jan 04, 2010
 */

#ifndef KINEMATIC_MODEL_WITH_LOCAL_CORRECTOR_H_
#define KINEMATIC_MODEL_WITH_LOCAL_CORRECTOR_H_

#include "kinematics/common/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

/*!
 * \class kinematic_model_with_local_corrector
 * \brief Abstract class with methods for local end-effector position correction with the use of local correction matrices.
 *
 * \author tkornuta
 * \date Jan 04, 2010
 */
class kinematic_model_with_local_corrector: public mrrocpp::kinematics::common::kinematic_model_with_tool
{
	protected:
		//! Calibration factor.
		double h;
		//! Correction vector.
		double V[6];
		//! Local correction matrix.
		double U[6][6];
		//! Local correction inverted matrix.
		double inv_U[6][6];

	public:
		//! Destructor - empty.
		virtual ~kinematic_model_with_local_corrector() { }

		//! Transformation related to the local correction matrix.
		virtual void local_corrector_transform(lib::Homog_matrix&);

		//! Inverse transformation related to the local correction matrix.
		virtual void local_corrector_inverse_transform(lib::Homog_matrix&);

		//! Computes external coordinates on the base of internal coordinates (i2e - internal to external).
		virtual void i2e_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame);

		//! Computes external coordinates on the base of internal coordinates, without the computations related with the attached tool.
		virtual void i2e_wo_tool_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame);

		//! Computes internal coordinates basing on external coordinates (e2i - external to internal).
		virtual void e2i_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame);

		//! Computes internal coordinates basing on external coordinates, without the computations related with the attached tool.
		virtual void e2i_wo_tool_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame);

};

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif /* KINEMATIC_MODEL_WITH_LOCAL_CORRECTOR_H_ */
