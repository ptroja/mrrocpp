#ifndef MP_ROBOT_H_
#define MP_ROBOT_H_

/*!
 * @file
 * @brief File contains mp base robot declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

// niezbedny naglowek z definiacja PROCESS_SPAWN_RSH
#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp_mp/ecp_mp_robot.h"

namespace mrrocpp {
namespace mp {

namespace task {
class task;
} // namespace task

namespace robot {

/*!
 * @brief Base class of all mp robots
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class robot : public ecp_mp::robot
{

private:
	/**
	 * @brief nummber of servos (joints)
	 */
	const int number_of_servos;

#if !defined(PROCESS_SPAWN_RSH)

	/**
	 * @brief node descriptor of spawned ECP process
	 */
	uint32_t nd;
#endif

	/**
	 * @brief pid of spawned ECP process
	 */
	pid_t ECP_pid;

#if !defined(USE_MESSIP_SRR)

	/**
	 * @brief main ECP communication channel descriptor
	 */
	int ECP_fd;
#else

	/**
	 * @brief main ECP communication channel descriptor
	 */
	messip_channel_t* ECP_fd;
#endif

protected:

	/**
	 * @brief mp taks object reference
	 */
	task::task &mp_object;

public:
	/**
	 * @brief continuous coordination flag
	 *
	 * if it set it causes every macrostep communication with ECP
	 */
	bool continuous_coordination;

	/**
	 * @brief sends pulse to ecp
	 *
	 * sends communication request etc.
	 * @param[in] pulse_code pulse code
	 * @param[in] pulse_value pusle value - default = -1
	 */
	void send_pulse_to_ecp(int pulse_code, int pulse_value = 1);

	/**
	 * @brief command buffer for ecp
	 *
	 * it is send during communication with ECP
	 */
	lib::MP_COMMAND_PACKAGE mp_command;

	/**
	 * @brief reply buffer from ecp
	 *
	 * it is received during communication with ECP
	 */
	lib::ECP_REPLY_PACKAGE ecp_reply_package;

	/**
	 * @brief ECP pulse receive time
	 *
	 * it is used to diversify macrostep length in multi continous coordination
	 * taking into account differences in communication readiness pulse receive time from different ECP's
	 */
	struct timespec ecp_pulse_receive_time;

	/**
	 * @brief the communication with EDP flag
	 *
	 * if the flag is set (default) the MP communicates with ECP in Move method of generator\n
	 * Sometimes it is needed to disable communication e.g. when there is a need to communicate only With MP or VSP\n
	 * in the following iterations of Move
	 */
	bool communicate_with_ecp;

	/**
	 * @brief reference to sr_ecp object for sending messages to UI_SR console
	 */
	lib::sr_ecp &sr_ecp_msg; // obiekt do komunikacji z SR

	/**
	 * @brief A server connection ID identifying ECP
	 */
	int ecp_scoid;

	/**
	 * @brief flag indicating opened pulse connection from ECP
	 */
	bool ecp_opened;

	/**
	 * @brief pulse code from ECP
	 */
	char ecp_pulse_code;

	/**
	 * @brief new pulse from ecp flag
	 */
	bool new_pulse;

	/**
	 * @brief new pulse from ecp checked flag
	 */
	bool new_pulse_checked;

	/**
	 * @brief constructor
	 * @param l_robot_name robot label
	 * @param _section_name ECP configuration file section
	 * @param mp_object_l mp task object reference
	 * @param _number_of_servos number of robot servos (joints)
	 */
			robot(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l, int _number_of_servos);

	/**
	 * @brief destructor
	 *
	 * it closes communication channels and kills ECP process
	 */
	virtual ~robot();

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
};

/*!
 * @brief MP robot error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class MP_error
{
public:

	/**
	 * @brief error class (type)
	 */
	const lib::error_class_t error_class;

	/**
	 * @brief error number
	 */
	const uint64_t error_no;

	/**
	 * @brief constructor
	 * @param err0 error class
	 * @param err1 error number
	 */
	MP_error(lib::error_class_t err0, uint64_t err1);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_ROBOT_H_*/
