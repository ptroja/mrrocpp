/*!
 * @file
 * @brief File contains mp base task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#ifndef MP_TASK_H_
#define MP_TASK_H_

#include <ostream>

#include "base/mp/mp_typedefs.h"
#include "base/ecp_mp/ecp_mp_task.h"

#include "base/lib/agent/InputBuffer.h"
#include "base/lib/agent/OutputBuffer.h"

namespace mrrocpp {
namespace mp {

// forward delcaration
namespace generator {
class generator;
}

namespace task {

/**
 * Two useful MP robot addition macros.
 * @note this is necessary to first create robot and then assign it to robot_m
 * the robot constructor can not be directly called with them associated robot_m field creation
 * because it uses robot_m
 * @note if the robot has been already added, then do nothing
 */

#define ACTIVATE_MP_ROBOT(__robot_name) \
	if(robot_m.find(lib::__robot_name::ROBOT_NAME) == robot_m.end()) {\
		if (config.exists_and_true ("is_active", "[edp_" #__robot_name "]")) {\
			robot::robot* created_robot = new robot::__robot_name(*this);\
			robot_m[lib::__robot_name::ROBOT_NAME] = created_robot;\
		}\
	}

#define ACTIVATE_MP_DEFAULT_ROBOT(__robot_name) \
	if(robot_m.find(lib::__robot_name::ROBOT_NAME) == robot_m.end()) {\
		if (config.exists_and_true ("is_active", "[edp_" #__robot_name "]")) {\
			robot::robot* created_robot = new robot::robot(lib::__robot_name::ROBOT_NAME, *this, 0);\
			robot_m[lib::__robot_name::ROBOT_NAME] = created_robot;\
		}\
	}

//! Macro for checking if robot is marked as activate in the config file
#define IS_MP_ROBOT_ACTIVE(__robot_name) \
	(config.exists_and_true ("is_active", "[edp_" #__robot_name "]"))

//! Type for optionally active input data buffer
template <class T>
class InputPtr : private boost::shared_ptr<InputBuffer<T> > {
	//! Underlying implementation of 'optional' concept
	typedef boost::shared_ptr<InputBuffer<T> > ptrType;

public:
	//! Create input buffer and register within an agent
	void Create(Agent & owner, const std::string & name, const T & default_value = T())
	{
		if(ptrType::get()) {
	        std::ostringstream tmp;
	        tmp << "optional Input buffer \"" << name << "\"already created";
			throw std::runtime_error(tmp.str());
		}
		ptrType::operator=((ptrType) new InputBuffer<T>(owner, name, default_value));
	}

	//! Reuse access operator from the underlying 'optional' concept type
	using ptrType::operator->;
};

//! Type for optionally inactive output data buffer
template <class T>
struct OutputPtr : private boost::shared_ptr<OutputBuffer<T> > {
	//! Underlying implementation of 'optional' concept
	typedef boost::shared_ptr<OutputBuffer<T> > ptrType;

public:
	//! Create input buffer and register within an agent
	void Create(RemoteAgent & owner, const std::string & name)
	{
		if(ptrType::get()) {
	        std::ostringstream tmp;
	        tmp << "optional Output buffer \"" << name << "\"already created";
			throw std::runtime_error(tmp.str());
		}
		ptrType::operator=((ptrType) new OutputBuffer<T>(owner, name));
	}

	//! Reuse access operator from the underlying 'optional' concept type
	using ptrType::operator->;
};

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
	 * @param robot_name robot to receive a command
	 */
	void set_next_ecp_state(const std::string & l_state, int l_variant, const char* l_string, int str_len, const lib::robot_name_t & robot_name);

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

	void wait_for_task_termination(bool activate_trigger, const std::vector<lib::robot_name_t> & robotSet);

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

	/**
	 * @brief Check if the robot has been created and activated
	 * @param name robot name
	 */
	bool is_robot_activated(const lib::robot_name_t & name) const
	{
		return ((robot_m.find(name) != robot_m.end()) ? true : false);
	}

private:
	friend class robot::robot;

	/**
	 * @brief pulse from UI
	 */
	InputBuffer<char> ui_pulse;
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
