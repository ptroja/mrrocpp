/**
 * \file smooth_trajectory_pose.h
 * \brief Header file for smooth_trajectory_pose
 *
 * Contains declaration of smooth_trajectory_pose class and its methods.
 */

#if !defined(_ECP_smooth_TRAJECTORY_POSE_H)
#define  _ECP_smooth_TRAJECTORY_POSE_H

#include "base/lib/com_buf.h"		// contains lib::POSE_SPECIFICATION
#include "base/lib/impconst.h"	// contains MAX_SERVOS_NR
namespace mrrocpp {
namespace ecp_mp {
namespace common {

/**
 * Class is a container used by smooth trajectory generator. One instance of the class contains information about one segment of the trajectory (single movement in one particular direction).
 */
class smooth_trajectory_pose
{
public:
	/**
	 * Representation used in the trajectory segment.
	 */
	lib::ECP_POSE_SPECIFICATION arm_type;
	/**
	 * Initial velocity for the pose, for each axis.
	 */
	double v_p[MAX_SERVOS_NR];
	/**
	 * Final velocity for the pose, for each axis.
	 */
	double v_k[MAX_SERVOS_NR];
	/**
	 * Maximal velocity for the movement, for each axis.
	 */
	double v[MAX_SERVOS_NR];
	/**
	 * Maximal acceleration for the movement, for each axis.
	 */
	double a[MAX_SERVOS_NR];
	/**
	 * Desired position
	 */
	double coordinates[MAX_SERVOS_NR];
	/**
	 * Number of the macrostep in which the first part of the movement ends (first out of three).
	 */
	double przysp[MAX_SERVOS_NR];
	/**
	 * Number of the macrostep in which the seconde part of the movement ends.
	 */
	double jedn[MAX_SERVOS_NR];
	/**
	 * Distance covered in the second part of the movement.
	 */
	double s_jedn[MAX_SERVOS_NR];
	/**
	 * Distance covered in the first part of the movement.
	 */
	double s_przysp[MAX_SERVOS_NR];
	/**
	 * Initial position for the pose.
	 */
	double start_position[MAX_SERVOS_NR];
	/**
	 * Direction of the movement. Either equal to 1 or -1.
	 */
	double k[MAX_SERVOS_NR];
	/**
	 * Maximal acceleration for the given segment (pose) (calculated, can be smaller or equal to a).
	 */
	double a_r[MAX_SERVOS_NR];
	/**
	 * Maximal velocity for the given segment (pose) (calculated, can be smaller or equal to v).
	 */
	double v_r[MAX_SERVOS_NR];
	/**
	 * Number of macrosteps in pose.
	 */
	int interpolation_node_no;
	/**
	 * Gripper velocity.
	 */
	double v_grip;
	/**
	 * Time needed to perform a movement.
	 */
	double t;
	/**
	 * Motion kinematic_model_with_tool.
	 */
	int model[MAX_SERVOS_NR];
	/**
	 * Number of the given position in whole trajectory chain.
	 */
	int pos_num;

	/**
	 * Empty constructor.
	 */
	smooth_trajectory_pose(void);
	/**
	 * Constructor which initiates some variables (those which can be found in the file containing trajectory).
	 * \param at representation used in the given pose
	 * \param coordinates desired position for all of the axes
	 * \param vv maximal velocities for the trajectory segment for all of the axes
	 * \param aa maximal accelerations for the trajectory segment for all of the axes
	 */
			smooth_trajectory_pose(lib::ECP_POSE_SPECIFICATION at, const double* coordinates, const double* vv, const double* aa);

private:
	// boost serialization methods
	friend class boost::serialization::access;

	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::make_nvp("coordinateType", arm_type);
		ar & boost::serialization::make_nvp("Velocity", v);
		ar & boost::serialization::make_nvp("Accelerations", a);
		ar & boost::serialization::make_nvp("Coordinates", coordinates);
	}
}; // end:class smooth_trajectory_pose


} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_smooth_TRAJECTORY_POSE_H */
