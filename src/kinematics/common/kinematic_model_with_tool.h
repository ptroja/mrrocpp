/*!
 * \file kinematic_model_with_tool.h
 * \brief File containing the declaration of kinematic_model_with_tool class.
 *
 * \author tkornuta
 * \date Jan 04, 2010
 */

#if !defined(__EDP_KIN_MODEL)
#define __EDP_KIN_MODEL

#include <string>
#include "lib/mrmath/mrmath.h"
#include "lib/impconst.h"
#include "kinematics/common/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

/*!
 * \class kinematic_model_with_tool
 * \brief Abstract class with methods for utilization in kinematic models with tools attached to its end-effectors and which contains methods for computations of transformation of robot base to global reference frame.
 *
 * \author tkornuta
 * \date Jan 04, 2010
 */
class kinematic_model_with_tool: public mrrocpp::kinematics::common::kinematic_model
{
	protected:
		//! Flag related to the computations to the global reference frame.
		bool global_frame_computations;
		//! Flag related to the attached tool computations.
		bool attached_tool_computations;

	public:

		//! Homogeneous matrix representing the transformation between the robot base and global reference frame.
		lib::Homog_matrix global_base;

		//! Homogeneous matrix representing the transformation between the end effector and tool attached to its end.
		lib::Homog_matrix tool;

		//! Class constructor - empty.
		kinematic_model_with_tool();

		//! Class virtual destructor - empty.
		virtual ~kinematic_model_with_tool();

		//! Computes external coordinates on the base of internal coordinates (i2e - internal to external).
		virtual void
				i2e_transform(const double* local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

		//! Computes external coordinates on the base of internal coordinates, without the computations related with the attached tool.
		virtual void
				i2e_wo_tool_transform(const double* local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

		//! Computes internal coordinates basing on external coordinates (e2i - external to internal).
		virtual void
				e2i_transform(double* local_desired_joints, double* local_current_joints, lib::Homog_matrix& local_desired_end_effector_frame);

		//! Computes internal coordinates basing on external coordinates, without the computations related with the attached tool.
		virtual void
				e2i_wo_tool_transform(double* local_desired_joints, double* local_current_joints, lib::Homog_matrix& local_desired_end_effector_frame);

		//! Computes robot base transformation to global reference frame.
		inline void global_frame_transform(lib::Homog_matrix&);

		//! Computes inverse base-global transformation - from global reference frame to robot base frame.
		inline void global_frame_inverse_transform(lib::Homog_matrix&);

		//! Computes transformation of end-effector frame to attached tool frame.
		inline void attached_tool_transform(lib::Homog_matrix&);

		//! Computes inverse end-effector-tool transformation.
		inline void attached_tool_inverse_transform(lib::Homog_matrix&);

};//: kinematic_model

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif
