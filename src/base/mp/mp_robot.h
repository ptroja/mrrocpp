/*!
 * @file
 * @brief File contains mp base robot declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#ifndef MP_ROBOT_H_
#define MP_ROBOT_H_

#include "base/lib/configurator.h"
#include "base/lib/child.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp_mp/ecp_mp_robot.h"
#include "base/lib/agent/RemoteAgent.h"
#include "base/lib/agent/InputBuffer.h"
#include "base/lib/agent/OutputBuffer.h"

namespace mrrocpp {
namespace mp {

namespace task {
class task_base;
class task;
} // namespace task
namespace generator {
class generator;
} // namespace generator

namespace robot {

/*!
 * @brief Base class of all mp robots
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class robot : public ecp_mp::robot
{
	// Both the generator and task have access to private methods
	friend class mrrocpp::mp::generator::generator;
	friend class mrrocpp::mp::task::task_base;

private:
	/**
	 * @brief nummber of servos (joints)
	 */
	const int number_of_servos;

	/**
	 * @brief pid of spawned ECP process
	 */
	lib::child ECP_pid;

public:
	//! Remote agent proxy
	lib::agent::RemoteAgent ecp;

private:
	//! Remote agent's data buffer
	lib::agent::OutputBuffer <lib::MP_COMMAND_PACKAGE> command;

	/**
	 * @brief reference to sr_ecp object for sending messages to UI_SR console
	 */
	lib::sr_ecp & sr_ecp_msg; // obiekt do komunikacji z SR

	/**
	 * @brief send a single command to the ECP
	 */
	void send_command(lib::MP_COMMAND);

	/**
	 * @brief executes the communication sequence with ECP with error handling
	 *
	 * called from move_method
	 */
	void execute_motion(void);

	/**
	 * @brief sends a message to terminate ECP task with error handling
	 */
	void terminate_ecp(void);

	/**
	 * @brief sends a message to start ECP task with error handling
	 */
	void start_ecp(void);

	/**
	 * @brief sends a message to pause ECP task
	 */
	void pause_ecp(void);

	/**
	 * @brief sends a message to resume ECP task
	 */
	void resume_ecp(void);

	/**
	 * @brief ECP errors handler
	 */
	void ecp_errors_handler();

public:
	/**
	 * @brief command buffer for ecp
	 *
	 * it is send during communication with ECP
	 */
	lib::MP_COMMAND_PACKAGE mp_command;

	//! Data buffer with messages from the ECP
	//! TODO: users should not use this data directly, only the 'const ecp_reply_package'.
	lib::agent::InputBuffer <lib::ECP_REPLY_PACKAGE> reply;

	/**
	 * @brief reply buffer from ecp
	 *
	 * it is received during communication with ECP
	 */
	const lib::ECP_REPLY_PACKAGE & ecp_reply_package;

	/**
	 * @brief the communication with EDP flag
	 *
	 * if the flag is set (default) the MP communicates with ECP in Move method of generator.
	 * Sometimes it is needed to disable communication, e.g. when there is a need to communicate only with MP or VSP.
	 * in the following iterations of Move.
	 */
	bool communicate_with_ecp;

	/**
	 * @brief constructor
	 * @param l_robot_name robot label
	 * @param mp_object_l mp task object reference
	 * @param _number_of_servos number of robot servos (joints)
	 */
	robot(const lib::robot_name_t & l_robot_name, task::task &mp_object_l, int _number_of_servos);

	/**
	 * @brief destructor
	 *
	 * it closes communication channels and kills ECP process
	 */
	virtual ~robot();
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_ROBOT_H_*/
