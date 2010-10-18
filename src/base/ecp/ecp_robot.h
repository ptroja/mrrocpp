#if !defined(_ECP_ROBOT_H)
#define _ECP_ROBOT_H

/*!
 * @file
 * @brief File contains ecp base robot declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/lib/com_buf.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/lib/configurator.h"
#include "base/ecp_mp/ecp_mp_robot.h"
#include "base/lib/single_thread_port.h"

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip.h"
#endif

class ui_common_robot;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class transparent;

}

namespace task {
class task;
} // namespace task

namespace robot {

/*!
 * @brief Base class of all ecp robots
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ecp_robot : public ecp_mp::robot
{
	// friend classes
	friend class ui_common_robot;
	friend class ecp::common::generator::transparent;

private:
	/**
	 * @brief method to directly copy mp command to edp buffer
	 *
	 * used e.g. in transparent generator in strict coordination
	 * @param[in] mp_buffer buffer including mp command
	 */
	void copy_mp_to_edp_buffer(const lib::c_buffer& mp_buffer);

	/**
	 * @brief method to directly copy edp reply to mp reply
	 *
	 * used e.g. in transparent generator in strict coordination
	 * @param[out] mp_buffer buffer including mp reply
	 */
	void copy_edp_to_mp_buffer(lib::r_buffer& mp_buffer);

	/**
	 * @brief method to spawn and connect to EDP
	 *
	 * when called from Ui it first spawns then connects to EDP,\n
	 * when called from ECP it only connects to existing EDP
	 * @param config configurator of communcation channels, edp binary file name etc.
	 */
	void connect_to_edp(lib::configurator &config);

	/**
	 * @brief pid of EDP process
	 */
	pid_t EDP_MASTER_Pid; // Identyfikator procesu driver'a edp_m

	/**
	 * @brief  the EDP spawn and kill flag
	 *
	 * if the flag is set the EDP is spawned with robot object creation then killed with destruction\n
	 * it is set when UI calls robot constructor
	 */
	const bool spawn_and_kill;

protected:

public:
	/**
	 * @brief to exchange data with generators
	 *
	 * it is used in some robots
	 */
	lib::single_thread_port_manager port_manager;

	/**
	 * @brief the communication with EDP flag
	 *
	 * if the flag is set (default) the ECP communicates with EDP in Move method of generator\n
	 * Sometimes it is needed to disable communication e.g. when there is a need to communicate only With MP or VSP\n
	 * in the following iterations of Move
	 */
	bool communicate_with_edp;

	/**
	 * @brief set the EDP command buffer from data_port structures
	 *
	 * currently it is executed only in sporadicly coordinated robots using data_ports
	 */
	virtual void create_command();
	/**
	 * @brief set the data_port structures from EDP reply buffer
	 *
	 * currently it is executed only in sporadicly coordinated robots using data_ports
	 */
	virtual void get_reply();

	/**
	 * @brief set the command and get reply from EDP
	 *
	 * it communicates directly with EDP
	 */
	void send();

	/**
	 * @brief Query EDP
	 *
	 * it first set the query flag in EDP command then calls send() method
	 */
	void query();

	/**
	 * @brief command send to EDP
	 */
	lib::ecp_command_buffer ecp_command;

	/**
	 * @brief reply received from EDP
	 */
	lib::r_buffer reply_package;

	/**
	 * @brief reference to sr_ecp object for sending messages to UI_SR console
	 */
	lib::sr_ecp & sr_ecp_msg; // obiekt do komunikacji z SR

	/**
	 * @brief flag if the robot is synchronised or not
	 */
	bool synchronised; // Flaga synchronizacji robota (true - zsynchronizowany, false - nie)

	/**
	 * @brief nummber of servos (joints)
	 */
	const int number_of_servos;

	/**
	 * @brief the configuration file section name of associated EDP process
	 */
	const std::string edp_section;

#if !defined(USE_MESSIP_SRR)

	/**
	 * @brief file descriptor of EDP communication chanell
	 */
	int EDP_fd; // by Y&W
#else

	/**
	 * @brief file descriptor of EDP communication chanell
	 */
	messip_channel_t *EDP_fd;
#endif

	/**
	 * @brief executed the communication sequence with EDP: set and query with error handling
	 *
	 * it can be reimplemented to maintain new error handling e.g.: in nose_run force generator
	 */
	virtual void execute_motion(void);

	/**
	 * @brief constructor called from UI
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	ecp_robot(lib::robot_name_t _robot_name, int _number_of_servos, const std::string &_edp_section, lib::configurator &_config, lib::sr_ecp &_sr_ecp);

	/**
	 * @brief constructor called from ECP
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _ecp_object ecp tak object reference
	 */
	ecp_robot(lib::robot_name_t _robot_name, int _number_of_servos, const std::string &_edp_section, common::task::task& _ecp_object);

	/**
	 * @brief returns EDP_MASTER_Pid - EDP pid
	 */
	pid_t get_EDP_pid(void) const;

	/**
	 * @brief desctructor
	 *
	 * it closes communication channels and optionally kills EDP process
	 */
	virtual ~ecp_robot(void);

	/**
	 * @brief send the synchronise command to EDP
	 *
	 * it also waits for reply status of synchronization completition
	 */
	void synchronise(void);

	/**
	 * @brief returns synchronised flag - synchronisation status
	 */
	bool is_synchronised(void) const; // Czy robot zsynchronizowany?

};

/*!
 * @brief ECP robot error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ECP_error
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
	 * @brief edp error structure
	 */
	lib::edp_error error;

	/**
	 * @brief constructor
	 * @param err_cl error class
	 * @param err_no error number
	 * @param err0 EDP error0 number
	 * @param err1 EDP error1 number
	 */
	ECP_error(lib::error_class_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
};

/*!
 * @brief ECP robot main error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ECP_main_error
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
	 * @param err_cl error class
	 * @param err_no error number
	 */
	ECP_main_error(lib::error_class_t err_cl, uint64_t err_no);
};

} // namespace robot
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_ROBOT_H */
