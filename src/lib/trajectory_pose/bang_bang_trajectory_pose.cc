/**
 * \file bang_bang_trajectory_pose.cc
 * \brief bang_bang_trajectory_pose class and its methods
 *
 * Contains bodies of the methods of bang_bang_trajectory_pose class.
 */

#include <string.h>

#include "lib/trajectory_pose/bang_bang_trajectory_pose.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

bang_bang_trajectory_pose::bang_bang_trajectory_pose(void)
{
}

bang_bang_trajectory_pose::bang_bang_trajectory_pose(lib::ECP_POSE_SPECIFICATION arm_type, const vector <double> & coordinates, const vector <
		double> & v, const vector <double> & a) :
	trajectory_pose(arm_type, coordinates)
{

	this->v = v;
	this->a = a;

	v_p = vector <double> (axes_num);
	v_k = vector <double> (axes_num);
	v_max = vector <double> (axes_num);
	przysp = vector <double> (axes_num);
	jedn = vector <double> (axes_num);
	s_jedn = vector <double> (axes_num);
	s_przysp = vector <double> (axes_num);
	start_position = vector <double> (axes_num);
	a_r = vector <double> (axes_num);
	v_r = vector <double> (axes_num);
	model = vector <int> (axes_num);
}

bang_bang_trajectory_pose::~bang_bang_trajectory_pose()
{

}

} // namespace trajectory_pose
} // namespace common
} // namespace ecp
} // namespace mrrocpp
