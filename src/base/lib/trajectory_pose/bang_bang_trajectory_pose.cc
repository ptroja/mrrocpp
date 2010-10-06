/**
 * @file
 * @brief Contains definitions of the methods of bang_bang_trajectory_pose class.
 * @author rtulwin
 * @ingroup generators
 */

#include <cstring>

#include "base/lib/trajectory_pose/bang_bang_trajectory_pose.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

using namespace std;

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
	a_max = vector <double> (axes_num);
	acc = vector <double> (axes_num);
	uni = vector <double> (axes_num);
	s_uni = vector <double> (axes_num);
	s_acc = vector <double> (axes_num);
	s_dec = vector <double> (axes_num);
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
