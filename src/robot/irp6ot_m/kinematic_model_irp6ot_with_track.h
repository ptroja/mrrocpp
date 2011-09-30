/*!
 * @file
 * @brief File containing the kinematic_model_irp6ot_with_track class.
 *
 * The kinematic_model_irp6ot_with_track kinematic model utilizes six out of seven IRP-6ot DOF - the newly added one is passive.
 *
 * @author tkornuta
 * @date 31.01.2007
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS irp6ot_m
 */

#if !defined(_IRP6OT_KIN_MODEL_WITH_TRACK)
#define _IRP6OT_KIN_MODEL_WITH_TRACK

#include "robot/irp6ot_m/kinematic_model_irp6ot_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot {

/*!
 *
 * @brief The kinematic model utilizes six out of seven IRP-6ot DOF - the newly added one is passive.
 *
 * @author tkornuta
 * @date 31.01.2007
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS
 */
class model_with_track : public model_with_wrist
{

public:
	/**
	 * @brief Constructor.
	 * @param _number_of_servos Number of servos (joints).
	 */
	model_with_track(int _number_of_servos);

	/**
	 * @brief Solves direct kinematics. The track-related DOF is active, while the new, additional DOF is passive.
	 * @param[in] local_current_joints Given internal (joints) values (d0, q1, q2, ...).
	 * @param[out] local_current_end_effector_frame Computed end-effector frame (a homogeneous matrix).
	 */
	virtual void
			direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix & local_current_end_effector_frame);

	/**
	 * @brief Solves inverse kinematics. The track-related DOF is active, while the new, additional DOF is passive.
	 * @param[out] local_desired_joints Computed join values (d0, q1, q2, ...).
	 * @param[in] local_current_joints Current (in fact previous) internal values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	virtual void
			inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

};

} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp

#endif
