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

bang_bang_trajectory_pose::bang_bang_trajectory_pose(const bang_bang_trajectory_pose &trj)
{
	this->v_p.insert(this->v_p.begin(), trj.v_p.begin(), trj.v_p.end());
	this->v_k.insert(this->v_k.begin(), trj.v_k.begin(), trj.v_k.end());
	this->v_max.insert(this->v_max.begin(), trj.v_max.begin(), trj.v_max.end());
	this->v.insert(this->v.begin(), trj.v.begin(), trj.v.end());

	this->v_r.insert(this->v_r.begin(), trj.v_r.begin(), trj.v_r.end());
	this->a_r.insert(this->a_r.begin(), trj.a_r.begin(), trj.a_r.end());
	this->a_max.insert(this->a_max.begin(), trj.a_max.begin(), trj.a_max.end());
	this->a.insert(this->a.begin(), trj.a.begin(), trj.a.end());
	this->acc.insert(this->acc.begin(), trj.acc.begin(), trj.acc.end());
	this->uni.insert(this->uni.begin(), trj.uni.begin(), trj.uni.end());
	this->s_uni.insert(this->s_uni.begin(), trj.s_uni.begin(), trj.s_uni.end());
	this->s_acc.insert(this->s_acc.begin(), trj.s_acc.begin(), trj.s_acc.end());
	this->s_dec.insert(this->s_dec.begin(), trj.s_dec.begin(), trj.s_dec.end());
	this->start_position.insert(this->start_position.begin(), trj.start_position.begin(), trj.start_position.end());
	this->model.insert(this->model.begin(), trj.model.begin(), trj.model.end());

	this->coordinates.insert(this->coordinates.begin(), trj.coordinates.begin(), trj.coordinates.end());
	this->k.insert(this->k.begin(), trj.k.begin(), trj.k.end());
	this->s.insert(this->s.begin(), trj.s.begin(), trj.s.end());
	this->times.insert(this->times.begin(), trj.times.begin(), trj.times.end());

	this->arm_type=trj.arm_type;
	this->interpolation_node_no=trj.interpolation_node_no;
	this->t=trj.t;
	this->pos_num=trj.pos_num;
	this->axes_num=trj.axes_num;
}

} // namespace trajectory_pose
} // namespace common
} // namespace ecp
} // namespace mrrocpp
