/*!
 * @file
 * @brief File contains mp base task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#ifndef MP_TASK_H_
#define MP_TASK_H_

#include <boost/foreach.hpp>

#include "base/mp/mp_task_base.h"

#include "generator/mp_g_set_next_ecps_state.h"
#include "generator/mp_g_wait_for_task_termination.h"
#include "generator/mp_g_send_end_motion_to_ecps.h"

namespace mrrocpp {
namespace mp {
namespace task {

/*!
 * @brief Base class of all mp tasks
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class task : public task_base
{
public:
	/**
	 * Constructor.
	 * @param _configurator configurator object
	 */
	task(lib::configurator &_config);

protected:
	/**
	 * @brief executes delay
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param _ms_delay delay time
	 */
	void wait_ms(unsigned int _ms_delay);

	/**
	 * @brief Insert robot names into generator's map (recursive variadic template).
	 * @param gen generator
	 * @param robot_name first robot name
	 * @param robot_names rest of robot names
	 */
	template<typename... Args>
	void insert_robots(generator::generator & gen, const lib::robot_name_t & robot_name, const Args&... robot_names)
	{
		// Insert head of the argument lists.
		gen.robot_m[robot_name] = this->robot_m[robot_name];

		// If there are more arguments...
		if(sizeof...(Args)) {
			// ...then handle them recursively.
			insert_robots(gen, robot_names...);
		}
	}

	/**
	 * @brief sends end motion command to ECP's
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param ... robots labels
	 */
	template<typename... Args>
	void send_end_motion_to_ecps(const lib::robot_name_t & robot_name, const Args&... robot_names)
	{
		generator::send_end_motion_to_ecps mp_semte_gen(*this);

		insert_robots(mp_semte_gen, robot_name, robot_names...);

		mp_semte_gen.Move();
	}

	/**
	 * @brief waits for task termination reply from set of robots ECP's
	 * it calls dedicated generator and then sends command in generator Move instruction
	 */
	void wait_for_task_termination(bool activate_trigger, const std::vector <lib::robot_name_t> & robotSet);

	/**
	 * @brief Wait for task termination with variadic list of robot names.
	 * @param activate_trigger
	 * @param robot_name
	 * @param robot_names
	 */
	template<typename... Args>
	void wait_for_task_termination(bool activate_trigger, const lib::robot_name_t & robot_name, const Args&... robot_names)
	{
		generator::wait_for_task_termination wtf_gen(*this);

		insert_robots(wtf_gen, robot_name, robot_names...);

		wtf_gen.configure(activate_trigger);

		wtf_gen.Move();
	}

	/**
	 * @brief sets the next state of ECP
	 * it calls dedicated generator and then sends new command in generator Move instruction
	 * @param l_state state label sent to ECP
	 * @param l_variant variant value sent to ECP
	 * @param l_string optional string sent to ECP
	 * @param str_len string length
	 * @param robot_name robot to receive a command
	 */
	template <typename BUFFER_TYPE>
	void set_next_ecp_state(const std::string & l_state, int l_variant, const BUFFER_TYPE & l_data, const lib::robot_name_t & robot_name)
	{
		// setting the next ecps state
		generator::set_next_ecps_state mp_snes_gen(*this);

		// Copy given robots to the map container
		mp_snes_gen.robot_m[robot_name] = robot_m[robot_name];

		mp_snes_gen.configure(l_state, l_variant, l_data);
		mp_snes_gen.Move();
	}

private:
	/**
	 * @brief Insert robot names into generator's map (recursive variadic template end-call).
	 * @param gen generator
	 * @param robot_name first robot name
	 */
	void insert_robots(generator::generator & gen, const lib::robot_name_t & robot_name)
	{
		// Insert head of the argument lists.
		gen.robot_m[robot_name] = this->robot_m[robot_name];
	}
};

/**
 * @brief returns inherited task pointer
 * @param _config configurator object reference
 * @return inherited task pointer
 */
task * return_created_mp_task(lib::configurator &_config);

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* MP_TASK_H_ */
