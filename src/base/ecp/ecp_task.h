#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

/*!
 * @file
 * @brief File contains ecp base task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <boost/ptr_container/ptr_unordered_map.hpp>
#include <boost/shared_ptr.hpp>

#include "base/lib/agent/Agent.h"
#include "base/ecp_mp/ecp_mp_task.h"
#include "base/ecp/ecp_robot.h"
#include "base/lib/agent/RemoteAgent.h"
#include "base/lib/agent/InputBuffer.h"
#include "base/lib/agent/OutputBuffer.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class generator_base;
}

namespace task {

/**
 * @brief Container type for storing ecp_generator objects.
 * @todo use boost::ptr_unrdered_map container
 *
 * @ingroup ecp
 */
typedef boost::unordered_map <lib::generator_name_t, generator::generator_base *> generators_t;

/**
 * @brief Type for Items from generators_t container.
 *
 * @ingroup ecp
 */
typedef generators_t::value_type generator_pair_t;

/*!
 * @brief Base class of all ecp tasks
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class task_base : public ecp_mp::task::task
{
private:
	/**
	 * @brief communication channels descriptors
	 */
	lib::fd_server_t trigger_attach;

	/**
	 * @brief Returns MP command type
	 * @return mp command variant
	 */
	lib::MP_COMMAND mp_command_type(void) const;

	/**
	 * @brief Initializes communication channels
	 */
	void initialize_communication(void);

protected:
	/**
	 * @brief Gets next state from MP
	 */
	void get_next_state(void);

	//! Type of the ECP to MP reply package
	typedef lib::ECP_REPLY_PACKAGE ecp_reply_t;

	//! Type of the MP to ECP command package
	typedef lib::MP_COMMAND_PACKAGE mp_command_t;

public:
	const boost::shared_ptr <robot::ecp_robot_base> & ecp_m_robot;

public:
	// TODO: following packages should be 'protected'
	/**
	 * @brief MP server proxy
	 */
	lib::agent::RemoteAgent MP;

	/**
	 * @brief Reply to MP
	 * @note This data type is task dependent, so it should be a parameter of a template class
	 */
	ecp_reply_t ecp_reply;

	//! Data buffer in the MP
	lib::agent::OutputBuffer <ecp_reply_t> reply;

	/**
	 * Data buffer with command from MP
	 *
	 * Buffer itself is a private object. Access to the data is provided with a 'const' access reference.
	 */
	lib::agent::InputBuffer <mp_command_t> command;

	/**
	 * @brief buffered MP command
	 * @note This data type is task dependent, so it should be a parameter of a template class
	 */
	const mp_command_t & mp_command;

	/**
	 * @brief buffered next state label sent by MP
	 */
	const std::string & mp_2_ecp_next_state_string;

	/**
	 * @brief ECP generators container
	 */
	generators_t generator_m;

	/**
	 * @brief continuous coordination flag
	 * influences generator Move method behavior
	 */
	bool continuous_coordination;

	/**
	 * @brief registers generator in generator_m
	 */
	void register_generator(generator::generator_base* _gen);

	/**
	 * @brief checks if new pulse arrived from UI on trigger channel
	 * @return pulse approached
	 */
	bool pulse_check();

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	task_base(lib::configurator &_config, boost::shared_ptr <robot::ecp_robot_base> & robot_ref);

	/**
	 * @brief Destructor
	 */
	virtual ~task_base();

	/**
	 * @brief main task algorithm
	 * it can be reimplemented in inherited classes
	 */
	virtual void main_task_algorithm(void);

	/**
	 * @brief method called from main_task_algorithm to handle next_state command from MP
	 * it can be reimplemented in inherited classes
	 * @todo remove this call together with deprecated attribute
	 */
	virtual void mp_2_ecp_next_state_string_handler(void) __attribute__ ((deprecated));

	/**
	 * @brief method called from main_task_algorithm to handle stop command from MP
	 * it can be reimplemented in inherited classes
	 */
	virtual void ecp_stop_accepted_handler(void);

	/**
	 * @brief sends the message to MP after task execution is completed
	 */
	void termination_notice(void);

	/**
	 * @brief Waits for START command from MP
	 */
	void wait_for_start(void);

	/**
	 * @brief Waits for STOP command from MP
	 */
	void wait_for_stop(void);

	/**
	 * @brief method called from main_task_algorithm to handle ecp subtasks execution
	 * it can be reimplemented in inherited classes
	 */
	void subtasks_and_generators_dispatcher();

public:
	// TODO: what follows should be private method or accessible only to some friend classes

	/**
	 * @brief Sets ECP reply type before communication with MP
	 */
	void set_ecp_reply(lib::ECP_REPLY ecp_r);

	/**
	 * @brief Receives MP message
	 * @return true if the END_MOTION has been received
	 */
	bool peek_mp_message();

	/**
	 * @brief waits for resume or stop command from MP
	 */
	void wait_for_resume();

	/**
	 * @brief informs if mp_2_ecp_next_state_string_handler is reimplemented in derrived classed
	 */
	bool mp_2_ecp_next_state_string_handler_active;

};

template <typename ECP_ROBOT_T>
class _task : public task_base
{
public:
	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	_task(lib::configurator &_config) :
			task_base(_config, (boost::shared_ptr <robot::ecp_robot_base> &) ecp_m_robot)
	{
	}

	/**
	 * @brief Destructor
	 */
	virtual ~_task()
	{
	}

	/**
	 * @brief Type of the associated robot
	 */
	typedef ECP_ROBOT_T robot_t;

	/**
	 * @brief Type of the specialized task class itself
	 */
	typedef _task <ECP_ROBOT_T> task_t;

	/**
	 * @brief Associated robot object shared pointer
	 */
	boost::shared_ptr <ECP_ROBOT_T> ecp_m_robot;
};

typedef _task <robot::ecp_robot> task;

task_base* return_created_ecp_task(lib::configurator &_config);

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */
