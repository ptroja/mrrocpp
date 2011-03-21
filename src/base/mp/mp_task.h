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
		if (config.value <int> ("is_active", "[edp_" #__robot_name "]")) {\
			robot::robot* created_robot = new robot::__robot_name(*this);\
			robot_m[lib::__robot_name::ROBOT_NAME] = created_robot;\
		}

#define ACTIVATE_MP_DEFAULT_ROBOT(__robot_name) \
		if (config.value <int> ("is_active", "[edp_" #__robot_name "]")) {\
			robot::robot* created_robot = new robot::robot(lib::__robot_name::ROBOT_NAME, lib::__robot_name::ECP_SECTION, *this, 0);\
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
	 */
	typedef enum _MP_RECEIVE_PULSE_ENUM
	{
		NONBLOCK, BLOCK
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
			set_next_ecps_state(std::string l_state, int l_variant, const char* l_string, int str_len, int number_of_robots, ...);

	/**
	 * @brief sends end motion command to ECP's
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param number_of_robots number of robots to receive command
	 * @param ... robots labels
	 */
	void send_end_motion_to_ecps(int number_of_robots, ...);

	/**
	 * @brief sends end motion command to ECP's - mkisiel xml task version
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param number_of_robots number of robots to receive command
	 * @param properRobotsSet pointer to robot list
	 */
	void send_end_motion_to_ecps(int number_of_robots, lib::robot_name_t *properRobotsSet);

	/**
	 * @brief runs extended empty generator
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param activate_trigger determines if mp_trigger finishes generator execution
	 * @param number_of_robots number of robots to receive command
	 * @param ... robots labels
	 */
	void run_extended_empty_gen_base(bool activate_trigger, int number_of_robots, ...);

	/**
	 * @brief runs extended empty generator and waits for task termination
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param number_of_robots_to_move determines if mp_trigger finishes generator execution
	 * @param number_of_robots_to_wait_for_task_termin number of robots to wait for task termination
	 * @param number_of_robots_to_move number of robots to move
	 * @param ... robots labels - first set, then second set
	 */
	void
	run_extended_empty_gen_and_wait(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ...);

	/**
	 * @brief runs extended empty generator and waits for task termination - mksiel xml version
	 * it calls dedicated generator and then sends command in generator Move instruction
	 * @param number_of_robots_to_move determines if mp_trigger finishes generator execution
	 * @param number_of_robots_to_wait_for_task_termin number of robots to wait for task termination
	 * @param number_of_robots_to_move number of robots to move
	 * @param robotsToMove robot to move list
	 * @param robotsWaitingForTaskTermination robot to wait list
	 */
	void
			run_extended_empty_gen_and_wait(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, lib::robot_name_t *robotsToMove, lib::robot_name_t *robotsWaitingForTaskTermination);

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
	void start_all(const common::robots_t & _robot_m);

	/**
	 * @brief termianted all ECP's
	 * it sends special MP command
	 */
	void terminate_all(const common::robots_t & _robot_m);

	/**
	 * @brief sends communication request pulse to ECP
	 */
	void request_communication_with_robots(const common::robots_t & _robot_m);

	/**
	 * @brief communicates with all ECP's that are set to communicate
	 */
	void execute_all(const common::robots_t & _robot_m);

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
	void mp_receive_ui_or_ecp_pulse(common::robots_t & _robot_m, generator::generator& the_generator);

private:
	friend class robot::robot;

	/**
	 * @brief wait until ECP/UI calls name_open() to pulse channel;]
	 * @return identifier (scoid/QNET or socket/messip) of the next connected process
	 */
	int wait_for_name_open(void);

	/**
	 * @brief A server connection ID identifying UI
	 */
	int ui_scoid;

	/**
	 * @brief flag indicating opened pulse connection from UI
	 */
	bool ui_opened;

	/**
	 * @brief code of the pulse received from UI
	 */
	char ui_pulse_code;

	/**
	 * @brief new UI pulse flaf
	 */
	bool ui_new_pulse;

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
