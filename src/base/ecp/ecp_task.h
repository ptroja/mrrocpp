#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

/*!
 * @file
 * @brief File contains ecp base task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <boost/shared_ptr.hpp>

#include "base/ecp_mp/ecp_mp_task.h"
#include "base/ecp/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace common {

//namespace robot {
//class ecp_robot;
//}

namespace sub_task {
class sub_task_base;
}

namespace task {

/**
 * @brief Container type for storing ecp_subtask objects.
 *
 * @ingroup ecp
 */
typedef std::map <std::string, sub_task::sub_task_base *> subtasks_t;

/**
 * @brief Type for Items from subtasks_t container.
 *
 * @ingroup ecp
 */
typedef subtasks_t::value_type subtask_pair_t;

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
	lib::fd_server_t ecp_attach, trigger_attach;

	/**
	 * @brief MP server communication channel descriptor to send pulses
	 */
	lib::fd_client_t MP_fd;

	/**
	 * @brief replies to MP message
	 * @param caller calling MP id
	 * @param mp_pulse_received MP pulse received flag
	 */
	bool reply_to_mp(int &caller, bool &mp_pulse_received);

	/**
	 * @brief sends pulse to MP to signal communication readiness
	 */
	void send_pulse_to_mp(int pulse_code, int pulse_value = 1);

	/**
	 * @brief Receives MP message
	 * @return caller (MP) ID
	 */
	int receive_mp_message(bool block);

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

public:
	// TODO: following packages should be 'protected'
	/**
	 * @brief Reply to MP
	 * @note This data type is task dependent, so it should be a parameter of a template class
	 */
	lib::ECP_REPLY_PACKAGE ecp_reply;

	/**
	 * @brief ECP subtasks container
	 */
	subtasks_t subtask_m;

	/**
	 * @brief buffered next state label sent by MP
	 */
	std::string mp_2_ecp_next_state_string;

	/**
	 * @brief buffered MP command
	 * @note This data type is task dependent, so it should be a parameter of a template class
	 */
	lib::MP_COMMAND_PACKAGE mp_command;

	/**
	 * @brief continuous coordination flag
	 * influences generator Move method behavior
	 */
	bool continuous_coordination;

	/**
	 * @brief checks if new pulse arrived from UI on trigger channel
	 * @return pulse approached
	 */
	bool pulse_check();

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	task_base(lib::configurator &_config);

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
	 */
	virtual void mp_2_ecp_next_state_string_handler(void);

	/**
	 * @brief method called from main_task_algorithm to handle stop command from MP
	 * it can be reimplemented in inherited classes
	 */
	virtual void ecp_stop_accepted_handler(void);

	/**
	 * @brief sends the message to MP after task execution is finished
	 */
	void ecp_termination_notice(void);

	/**
	 * @brief Waits for START command from MP
	 */
	void ecp_wait_for_start(void);

	/**
	 * @brief Waits for STOP command from MP
	 */
	void ecp_wait_for_stop(void);

	/**
	 * @brief method called from main_task_algorithm to handle ecp subtasks execution
	 * it can be reimplemented in inherited classes
	 */
	void subtasks_conditional_execution();

	/**
	 * @brief method to wait to receive pulses and messages from MP
	 * it returns when message is received (randevous happens)
	 */
	void wait_for_randevous_with_mp(int &caller, bool &mp_pulse_received);

public:
	// TODO: what follows should be private

	/**
	 * @brief communicates with MP
	 * @return true if MP sended NEXT_POSE, false if MP sended END_MOTION command
	 */
	bool mp_buffer_receive_and_send(void);

	/**
	 * @brief Sets ECP reply type before communication with MP
	 */
	void set_ecp_reply(lib::ECP_REPLY ecp_r);
};

template<typename ECP_ROBOT_T>
class _task : public task_base
{
public:
	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	_task(lib::configurator &_config) : task_base(_config)
	{}

	/**
	 * @brief Destructor
	 */
	virtual ~_task()
	{}

	/**
	 * @brief Type of the associated robot
	 */
	typedef ECP_ROBOT_T robot_t;

	/**
	 * @brief Type of the specialized task class itself
	 */
	typedef _task<ECP_ROBOT_T> task_t;

	/**
	 * @brief Associated single robot object shared pointer
	 */
	boost::shared_ptr<ECP_ROBOT_T> ecp_m_robot;
};

typedef _task<robot::ecp_robot> task;

task_base* return_created_ecp_task(lib::configurator &_config);

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */
