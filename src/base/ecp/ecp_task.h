#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

/*!
 * @file
 * @brief File contains ecp base task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp_mp/ecp_mp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace robot {
class ecp_robot;
}
namespace sub_task {

class sub_task;
}
namespace task {

/**
 * @brief Container type for storing ecp_subtask objects.
 *
 * @ingroup ecp
 */
typedef std::map <std::string, sub_task::sub_task *> subtasks_t;

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
class task : public ecp_mp::task::task
{
private:
#if !defined(USE_MESSIP_SRR)
	/**
	 * @brief communication channels descriptors
	 */
	name_attach_t *ecp_attach, *trigger_attach; // by Y

	/**
	 * @brief MP server communication channel descriptor to send pulses
	 */
	int MP_fd;
#else
	messip_channel_t *ecp_attach, *trigger_attach, *MP_fd;
#endif

	/**
	 * @brief sends pulse to MP to signalize communication readiness
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
	 */
	lib::ECP_REPLY_PACKAGE ecp_reply;

	/**
	 * @brief ECP subtasks stl map
	 */
	subtasks_t subtask_m;

	/**
	 * @brief buffered next state label sent by MP
	 */
	std::string mp_2_ecp_next_state_string;

	/**
	 * @brief buffered MP command
	 */
	lib::MP_COMMAND_PACKAGE mp_command;

	/**
	 * @brief associated single robot object pointer
	 */
	robot::ecp_robot* ecp_m_robot;

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
	task(lib::configurator &_config);

	/**
	 * @brief Destructor
	 */
	virtual ~task();

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

public:
	// TODO: what follows should be private method

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

task* return_created_ecp_task(lib::configurator &_config);

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */
