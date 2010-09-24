#if !defined(_ECP_TAUGHT_IN_POSE_H)
#define  _ECP_TAUGHT_IN_POSE_H

/*!
 * @file
 * @brief File contains ecp_taught_in_pose declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/lib/com_buf.h"		// lib::POSE_SPECIFICATION
#include "base/lib/impconst.h"	// lib::MAX_SERVOS_NR
namespace mrrocpp {
namespace ecp {
namespace common {

/*!
 * @brief class to store single position
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ecp_taught_in_pose
{
public:
	/**
	 * @brief position (pose) representation
	 */
	lib::ECP_POSE_SPECIFICATION arm_type;

	/**
	 * @brief motion duration
	 */
	double motion_time;

	/**
	 * @brief position (pose) coordinates vector
	 */
	double coordinates[lib::MAX_SERVOS_NR];

	/**
	 * @brief extra information associated with pose
	 */
	int extra_info;

	/**
	 * @brief Constructor
	 */
	ecp_taught_in_pose(void);

	/**
	 * @brief Constructor
	 * @param at position (pose) representation
	 * @param mt motion duration
	 * @param c position (pose) coordinates vector
	 * @param e_info extra info
	 */
	ecp_taught_in_pose(lib::ECP_POSE_SPECIFICATION at, double mt, const double c[lib::MAX_SERVOS_NR], int e_info = 0);
}; // end:class ecp_taught_in_pose

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TAUGHT_IN_POSE_H */
