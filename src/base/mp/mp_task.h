/*!
 * @file
 * @brief File contains mp base task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#ifndef MP_TASK_H_
#define MP_TASK_H_

#include "base/mp/mp_typedefs.h"
#include "base/ecp_mp/ecp_mp_task.h"

#include "base/lib/agent/DataBuffer.h"
#include "base/lib/messip/messip.h"

namespace mrrocpp {
namespace mp {

// forward delcaration
namespace generator {
class generator;
}

namespace task {

/*
 * Two usefull mp robot addition macros
 * this is necessary to first create robot and then assign it to robot_m
 * the robot constructor can not be directly called with them associated robot_m field creation\n
 * because it uses robot_m
 */

#define ACTIVATE_MP_ROBOT(__robot_name) \
		if (config.exists_and_true ("is_active", "[edp_" #__robot_name "]")) {\
			robot::robot* created_robot = new robot::__robot_name(*this);\
			robot_m[lib::__robot_name::ROBOT_NAME] = created_robot;\
		}

#define ACTIVATE_MP_DEFAULT_ROBOT(__robot_name) \
		if (config.exists_and_true ("is_active", "[edp_" #__robot_name "]")) {\
			robot::robot* created_robot = new robot::robot(lib::__robot_name::ROBOT_NAME, *this, 0);\
			robot_m[lib::__robot_name::ROBOT_NAME] = created_robot;\
		}

/*!
 * @brief Base class of all mp tasks
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class task : public ecp_mp::task::task
{
private:
	/**
	 * @brief Initializes communication channels
	 */
	void initialize_communication(void);

public:
	/**
	 * @brief communication channels descriptors
	 */
	static lib::fd_server_t mp_pulse_attach;

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	task(lib::configurator &_config);

	/**
	 * @brief Destructor
	 */
	virtual ~task(void);

	/**
	 * @brief pure virtual method to create robots
	 * it have to be reimplemented in inherited classes
	 */
	virtual void create_robots(void) = 0;

	/**
	 * @brief Waits for stop pulse from UI and terminated all ECP's
	 */
	void stop_and_terminate(void);

	/**
	 * @brief Enum of two possible pulse receive variants (BLOCK/NONBLOCK)
	 * @note it is assumend, that values of this enum equals to "block?" predicate.
	 */
	typedef enum _MP_RECEIVE_PULSE_ENUM
	{
		NONBLOCK = 0, BLOCK = 1
	} RECEIVE_PULSE_MODE;

	/**
	 * @brief Enum of three possible variants of pulse origin processes
	 */
	typedef enum _WAIT_FOR_NEW_PULSE_ENUM
	{
		NEW_ECP_PULSE, NEW_UI_PULSE, NEW_UI_OR_ECP_PULSE
	} WAIT_FOR_NEW_PULSE_MODE;

	/**
	 * @brief sets the goal pf player controlled robot
	 * @param robot_l robot label
	 * @param goal motion goal
	 */
	void set_next_playerpos_goal(lib::robot_name_t robot_l, const lib::playerpos_goal_t &goal);

	/**
	 * @brief sets the next state of ECP
	 * it calls dedicated generator and then sends new command in generator Move instruction
	 * @param l_state state label sent to ECP
	 * @param l_variant variant value sent to ECP
	 * @param l_string optional string sent to ECP
	 * @param str_len string length
	 * @param number_of_robots number of robots to receive command
	 * @param ... robots labels
	 */
	void
			set_next_ecps_state(const std::string & l_state, int l_variant, const char* l_string, int str_len, int number_of_robots, ...);

	/**
	 * @brief sends end motion command to ECP's
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param number_of_robots number of robots to receive command
	 * @param ... robots labels
	 */
	void send_end_motion_to_ecps(int number_of_robots, ...);

	/**
	 * @brief waits for task termination reply from set of robots ECP's
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param number_of_robots number of robots to receive command
	 * @param ... robots labels
	 */
	void wait_for_task_termination(bool activate_trigger, int number_of_robots, ...);

	/**
	 * @brief sends end motion command to ECP's - mkisiel xml task version
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param number_of_robots number of robots to receive command
	 * @param properRobotsSet pointer to robot list
	 */
	void send_end_motion_to_ecps(int number_of_robots, lib::robot_name_t *properRobotsSet);

	/**
	 * @brief executes delay
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param _ms_delay delay time
	 */
	void wait_ms(int _ms_delay); // zamiast delay

	/**
	 * @brief waits for START pulse from UI
	 */
	void wait_for_start(void);// by Y&W

	/**
	 * @brief waits for STOP pulse from UI
	 */
	void wait_for_stop(void);// by Y&W dodany tryb


	/**
	 * @brief starts all ECP's
	 * it sends special MP command
	 */
	void start_all();

	/**
	 * @brief pause all ECP's
	 * it sends special MP command
	 */
	void pause_all();

	/**
	 * @brief resume all ECP's
	 * it sends special MP command
	 */
	void resume_all();

	/**
	 * @brief termianted all ECP's
	 * it sends special MP command
	 */
	void terminate_all();

	/**
	 * @brief waits for acknowledge reply from all robots
	 */
	void wait_for_all_robots_acknowledge();

	/**
	 * @brief main task algorithm
	 * to implement ni inherited classes
	 */
	virtual void main_task_algorithm(void) = 0;

	/**
	 * @brief map of all robots used in the task
	 */
	common::robots_t robot_m;

	/**
	 * @brief receives pulse from UI or ECP
	 */
	void receive_ui_or_ecp_message(generator::generator& the_generator);

private:
	friend class robot::robot;

	/**
	 * @brief pulse from UI
	 */
	InputBuffer <char> ui_pulse;

	/**
	 * @brief checks new pulses from ECP and UI that already approach and optionally waits for pulse approach
	 * @param process_type - process type to wait for the pulses
	 * @param desired_wait_mode - decides if the method should wait for desired pulse or not
	 * @return desired pulse found
	 */
	bool
			check_and_optional_wait_for_new_pulse(WAIT_FOR_NEW_PULSE_MODE process_type, const RECEIVE_PULSE_MODE desired_wait_mode);
};

/**
 * @brief returns inherited task pointer
 * @param _config configurator object reference
 * @return inherited task pointer
 */
task* return_created_mp_task(lib::configurator &_config);

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /*MP_TASK_H_*/
