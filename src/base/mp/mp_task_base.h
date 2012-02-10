/*!
 * @file
 * @brief File contains mp base task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#ifndef MP_TASK_BASE_H_
#define MP_TASK_BASE_H_

#include <ostream>

#include "base/mp/mp_typedefs.h"
#include "base/ecp_mp/ecp_mp_task.h"

#include "base/lib/agent/InputBuffer.h"
#include "base/lib/agent/OutputBuffer.h"

namespace mrrocpp {
namespace mp {

// Forward declaration.
namespace generator {
class generator;
}

namespace task {

/**
 * MP robot utilities (macros and classes).
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
class InputPtr : private boost::shared_ptr <lib::agent::InputBuffer <T> >
{
	//! Underlying implementation of 'optional' concept
	typedef boost::shared_ptr <lib::agent::InputBuffer <T> > ptrType;

public:
	//! Create input buffer and register within an agent
	void Create(lib::agent::Agent & owner, const std::string & name, const T & default_value = T())
	{
		if (ptrType::get()) {
			std::ostringstream tmp;
			tmp << "optional Input buffer \"" << name << "\"already created";
			throw std::runtime_error(tmp.str());
		}

		ptrType::operator=((ptrType) new lib::agent::InputBuffer<T>(owner, name, default_value));
	}

	//! Reuse access operator from the underlying 'optional' concept type
	using ptrType::operator->;

	//! Reuse getter from the underlying 'optional' concept type
	using ptrType::get;
};

//! Type for optionally inactive output data buffer
template <class T>
struct OutputPtr : private boost::shared_ptr <lib::agent::OutputBuffer <T> >
{
	//! Underlying implementation of 'optional' concept
	typedef boost::shared_ptr <lib::agent::OutputBuffer <T> > ptrType;

public:
	//! Create input buffer and register within an agent
	void Create(lib::agent::RemoteAgent & owner, const std::string & name)
	{
		if (ptrType::get()) {
			std::ostringstream tmp;
			tmp << "optional Output buffer \"" << name << "\"already created";
			throw std::runtime_error(tmp.str());
		}

		ptrType::operator=((ptrType) new lib::agent::OutputBuffer<T>(owner, name));
	}

	//! Reuse access operator from the underlying 'optional' concept type
	using ptrType::operator->;

	//! Reuse getter from the underlying 'optional' concept type
	using ptrType::get;
};

/*!
 * @brief Base class of all mp tasks
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class task_base : public ecp_mp::task::task
{
private:
	/**
	 * @brief Initializes communication channels
	 */
	void initialize_communication(void);

	/**
	 * @brief waits for acknowledge reply from all robots
	 */
	void wait_for_all_robots_acknowledge();

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

protected:
	/**
	 * @brief Check if the robot has been created and activated
	 * @param name robot name
	 */
	bool is_robot_activated(const lib::robot_name_t & name) const
	{
		return (robot_m.find(name) != robot_m.end());
	}

public:
	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	task_base(lib::configurator &_config);

	/**
	 * @brief Destructor
	 */
	virtual ~task_base(void);

	/**
	 * @brief pure virtual method to create robots
	 * it have to be reimplemented in inherited classes
	 */
	virtual void create_robots(void) = 0;

	/**
	 * @brief waits for START pulse from UI
	 */
	void wait_for_start(void);

	/**
	 * @brief starts all ECP's
	 * it sends special MP command
	 */
	void start_all();

	/**
	 * @brief waits for STOP pulse from UI
	 */
	void wait_for_stop(void);

	/**
	 * @brief Waits for stop pulse from UI and terminated all ECP's
	 */
	void stop_and_terminate(void);

	/**
	 * @brief termianted all ECP's
	 * it sends special MP command
	 */
	void terminate_all();

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

protected:
	/**
	 * @brief pulse from UI
	 */
	lib::agent::InputBuffer<char> ui_pulse;
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* MP_TASK_BASE_H_ */
