#if !defined(_ECP_GEN_FORCE_H)
#define _ECP_GEN_FORCE_H

/*!
 * @file
 * @brief File contains force generators declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "generator/ecp/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class bias_edp_force : public common::generator::generator
{
public:
	// konstruktor
	bias_edp_force(common::task::task& _ecp_task);

	virtual bool first_step();
	virtual bool next_step();
};

// --------------------------------------------------------------------------
// Generator do lapania kostki

class tff_rubik_grab : public common::generator::generator
{
protected:
	lib::trajectory_description td;

	// do konfiguracji pracy generatora
	double goal_position, position_increment;
	unsigned int min_node_counter;
	bool both_axes_running;
	double desired_absolute_gripper_coordinate;

public:
	const int step_no;

	// konstruktor
	tff_rubik_grab(common::task::task& _ecp_task, int step = 0);

	void
			configure(double l_goal_position, double l_position_increment, unsigned int l_min_node_counter, bool l_both_axes_running =
					true);

	bool first_step();
	bool next_step();

}; // end : class ecp_tff_rubik_grab_generator


// --------------------------------------------------------------------------
// Generator do obracania sciany kostki

class tff_rubik_face_rotate : public common::generator::generator
{
protected:
	lib::trajectory_description td;

	// do konfiguracji pracy generatora

	double stored_gamma, turn_angle;
	bool range_change;

public:
	const int step_no;

	// konstruktor
	tff_rubik_face_rotate(common::task::task& _ecp_task, int step = 0);

	void configure(double l_turn_angle);

	bool first_step();
	bool next_step();

}; // end : class ecp_tff_rubik_face_rotate_generator


// --------------------------------------------------------------------------
// Generator do nasuniecia chwytaka na kostke

class tff_gripper_approach : public common::generator::generator
{
protected:

	lib::trajectory_description td;

	// do konfiguracji pracy generatora
	double speed;
	unsigned int motion_time;

public:
	const int step_no;

	// konstruktor
	tff_gripper_approach(common::task::task& _ecp_task, int step = 0);

	void configure(double l_speed, unsigned int l_motion_time);

	bool first_step();
	bool next_step();

}; // end : class ecp_tff_gripper_approach_generator

class force_tool_change : public common::generator::generator
{
protected:
	double tool_parameters[3]; // zobaczyc jeszcze co z tymi parametrami jak to bedzie w przypadku tego generatora
	double weight;
public:
	force_tool_change(common::task::task& _ecp_task);
	//ecp_force_tool_change_generator(common::task::task& _ecp_task, bool _is_synchronised, bool _debug);
	void set_tool_parameters(double x, double y, double z, double weight); // tez zobaczyc jakie tu mamy parametry

	bool first_step();
	bool next_step();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
