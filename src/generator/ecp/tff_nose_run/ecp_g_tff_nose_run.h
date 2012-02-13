#if !defined(_ECP_GEN_TFF_NOSE_RUN_H)
#define _ECP_GEN_TFF_NOSE_RUN_H

/*!
 * @file
 * @brief File contains tff nose run generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "ecp_mp_g_tff_nose_run.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief generator to move manipulator end-effector by exerting force to the tool
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class tff_nose_run : public common::generator::generator
{
protected:

	/**
	 * @brief if set the trigger pulse from UI finishes generator
	 */
	bool pulse_check_activated;

	/**
	 * @brief if set the measured force is displayed
	 */
	bool force_meassure;

	/**
	 * @brief stores desired EDP behavior
	 */
	struct generator_edp_data_type
	{
		double next_inertia[6], next_reciprocal_damping[6];
		double next_velocity[lib::MAX_SERVOS_NR], next_force_xyz_torque_xyz[6];
		lib::BEHAVIOUR_SPECIFICATION next_behaviour[6];
	} generator_edp_data;

public:

	/**
	 * @brief the number of macrostep steps
	 */
	const int step_no;

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 * @param step the number of macrostep steps (optional)
	 */
	tff_nose_run(common::task::task& _ecp_task, int step = 0);

	/**
	 * @brief communicates with EDP
	 * here it and handles kinematic exceptions to achieve constant motion without generator interuption after such an exception
	 */
	void execute_motion(void);

	/**
	 * @brief sets pulse check flag
	 * @param pulse_check_activated_l desired pulse check flag value
	 */
	void configure_pulse_check(bool pulse_check_activated_l);

	/**
	 * @brief sets force measure flag
	 * @param fm desired force measure flag value
	 */
	void set_force_meassure(bool fm);

	/**
	 * @brief Sets behavior parameters for all directions
	 * @param x linear x direction
	 * @param y linear y direction
	 * @param z linear z direction
	 * @param ax angular x direction
	 * @param ay angular y direction
	 * @param az angular z direction
	 */
	void
	configure_behaviour(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z, lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az);

	/**
	 * @brief Sets desired velocity for all directions
	 * @param x linear x direction
	 * @param y linear y direction
	 * @param z linear z direction
	 * @param ax angular x direction
	 * @param ay angular y direction
	 * @param az angular z direction
	 */
	void configure_velocity(double x, double y, double z, double ax, double ay, double az);

	/**
	 * @brief Sets desired force for all directions
	 * @param x linear x direction
	 * @param y linear y direction
	 * @param z linear z direction
	 * @param ax angular x direction
	 * @param ay angular y direction
	 * @param az angular z direction
	 */
	void configure_force(double x, double y, double z, double ax, double ay, double az);

	/**
	 * @brief Sets desired reciprocal of damping for all directions
	 * @param x linear x direction
	 * @param y linear y direction
	 * @param z linear z direction
	 * @param ax angular x direction
	 * @param ay angular y direction
	 * @param az angular z direction
	 */
	void configure_reciprocal_damping(double x, double y, double z, double ax, double ay, double az);

	/**
	 * @brief Sets desired inertia for all directions
	 * @param x linear x direction
	 * @param y linear y direction
	 * @param z linear z direction
	 * @param ax angular x direction
	 * @param ay angular y direction
	 * @param az angular z direction
	 */
	void configure_inertia(double x, double y, double z, double ax, double ay, double az);

	bool first_step();
	bool next_step();

	void conditional_execution();


};
// end : class ecp_tff_nose_run_generator

}// namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
