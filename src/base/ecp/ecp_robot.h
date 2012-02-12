#if !defined(_ECP_ROBOT_H)
#define _ECP_ROBOT_H

/*!
 * @file
 * @brief File contains ecp base robot declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <cerrno>

#include "ecp_exceptions.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp_mp/ecp_mp_robot.h"
#include "base/lib/single_thread_port.h"

#include "base/lib/messip/messip_dataport.h"

class ui_common_robot;

namespace mrrocpp {

namespace lib {
class sr_ecp;
struct c_buffer;
struct r_buffer;
class configurator;
}

namespace ecp {
namespace common {

namespace generator {
class transparent;
}

namespace task {
class task_base;
}

namespace robot {

/*!
 * @brief Base class of all ecp robots
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ecp_robot_base : public ecp_mp::robot
{
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
	 * @brief set the command and get reply from EDP
	 *
	 * it communicates directly with EDP
	 */
	virtual void send() = 0;

	/**
	 * @brief Query EDP
	 *
	 * it first set the query flag in EDP command then calls send() method
	 */
	virtual void query() = 0;

	/**
	 * @brief reference to object for sending messages to UI/SR console
	 */
	lib::sr_ecp & sr_ecp_msg;

	/**
	 * @brief flag if the robot is synchronised or not
	 */
	bool synchronised;

	/**
	 * @brief nummber of servos (joints)
	 */
	const int number_of_servos;

	/**
	 * @brief the configuration file section name of associated EDP process
	 */
	const std::string edp_section;

	/**
	 * @brief states if any data_port is set
	 */
	bool is_new_data;

	/**
	 * @brief states if any data_request_port is set
	 */
	bool is_new_request;

	/**
	 * @brief states if any data_ports_are_used
	 */
	bool data_ports_used;

	/**
	 * @brief executed the communication sequence with EDP: set and query with error handling
	 *
	 * it can be reimplemented to maintain new error handling e.g.: in nose_run force generator
	 */
	virtual void execute_motion(void) = 0;

	/**
	 * @brief constructor called from UI
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	ecp_robot_base(const lib::robot_name_t & _robot_name, int _number_of_servos, lib::configurator &_config, lib::sr_ecp &_sr_ecp);

	/**
	 * @brief constructor called from ECP
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _ecp_object ecp tak object reference
	 */
	ecp_robot_base(const lib::robot_name_t & _robot_name, int _number_of_servos, common::task::task_base& _ecp_object);

	/**
	 * @brief desctructor
	 *
	 * Closes communication channels and optionally kills EDP process
	 */
	virtual ~ecp_robot_base();

	/**
	 * @brief send the synchronise command to EDP
	 *
	 * Also waits for reply status of synchronization completion
	 */
	virtual void synchronise(void) = 0;

	/**
	 * @brief returns synchronised flag - synchronisation status
	 */
	bool is_synchronised(void) const;

	/**
	 * @brief returns EDP_MASTER_Pid - EDP pid
	 */
	pid_t get_EDP_pid(void) const;

protected:
	/**
	 * @brief file descriptor of EDP communication channel
	 */
	lib::fd_client_t EDP_fd;

private:
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
	pid_t EDP_pid;
};

class common_buffers_ecp_robot : public ecp_robot_base
{

public:
	friend class ecp::common::generator::transparent;

	/**
	 * @brief constructor called from UI
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	common_buffers_ecp_robot(const lib::robot_name_t & _robot_name, int _number_of_servos, lib::configurator &_config, lib::sr_ecp &_sr_ecp, lib::c_buffer & c_buffer_ref, lib::r_buffer & r_buffer_ref);

	/**
	 * @brief constructor called from ECP
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _ecp_object ecp tak object reference
	 */
	common_buffers_ecp_robot(const lib::robot_name_t & _robot_name, int _number_of_servos, common::task::task_base& _ecp_object, lib::c_buffer & c_buffer_ref, lib::r_buffer & r_buffer_ref);

	/**
	 * @brief desctructor
	 */
	virtual ~common_buffers_ecp_robot();

	/*!
	 * \brief Reference to base types of instruction
	 *
	 * The particular type is the field of derived classes
	 */
	lib::c_buffer & ecp_command;

	/*!
	 * \brief Reference to base types of reply
	 *
	 * The particular type is the field of derived classes
	 */
	lib::r_buffer & reply_package;
};

template <typename ROBOT_COMMAND_T = lib::c_buffer, typename ROBOT_REPLY_T = lib::r_buffer>
class _ecp_robot : public common_buffers_ecp_robot
{
public:
	friend class ecp::common::generator::transparent;

private:
	/**
	 * @brief method to directly copy mp command to edp buffer
	 *
	 * used e.g. in transparent generator in strict coordination
	 * @param[in] mp_buffer buffer including mp command
	 */
	void copy_mp_to_edp_buffer(const ROBOT_COMMAND_T & mp_buffer)
	{
		ecp_command = mp_buffer;
	}

	/**
	 * @brief method to directly copy edp reply to mp reply
	 *
	 * used e.g. in transparent generator in strict coordination
	 * @param[out] mp_buffer buffer including mp reply
	 */
	void copy_edp_to_mp_buffer(ROBOT_REPLY_T & mp_buffer)
	{
		mp_buffer = reply_package;
	}

public:
	/**
	 * @brief constructor called from UI
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	_ecp_robot(const lib::robot_name_t & _robot_name, int _number_of_servos, lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
			common_buffers_ecp_robot(_robot_name, _number_of_servos, _config, _sr_ecp, ecp_command, reply_package)
	{
	}

	/**
	 * @brief constructor called from ECP
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _ecp_object ecp tak object reference
	 */
	_ecp_robot(const lib::robot_name_t & _robot_name, int _number_of_servos, common::task::task_base& _ecp_object) :
			common_buffers_ecp_robot(_robot_name, _number_of_servos, _ecp_object, ecp_command, reply_package)
	{
	}

	/**
	 * @brief desctructor
	 *
	 * Calls base class destructor
	 */
	virtual ~_ecp_robot()
	{
	}

	/**
	 * @brief command to EDP
	 */
	ROBOT_COMMAND_T ecp_command;

	/**
	 * @brief Type of the command to be sent to a robot
	 */
	typedef ROBOT_COMMAND_T robot_command_t;

	/**
	 * @brief Reply from EDP
	 */
	ROBOT_REPLY_T reply_package;

	/**
	 * @brief Type of the reply from a robot
	 */
	typedef ROBOT_REPLY_T robot_reply_t;

	/**
	 * @brief set the EDP command buffer from data_port structures
	 *
	 * currently it is executed only in sporadically coordinated robots using data_ports
	 */
	virtual void create_command()
	{
	}

	/**
	 * @brief set the data_port structures from EDP reply buffer
	 *
	 * currently it is executed only in sporadically coordinated robots using data_ports
	 */
	virtual void get_reply()
	{
	}

	/**
	 * @brief executed the communication sequence with EDP: set and query with error handling
	 *
	 */
	void execute_motion(void)
	{
		send();

		if (reply_package.reply_type == lib::ERROR) {
			query();
			BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));
		}

		query();

		if (reply_package.reply_type == lib::ERROR) {

			BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));
		}
	}

	void query()
	{
		ecp_command.instruction_type = lib::QUERY;
		send(); // czyli wywolanie funkcji ecp_buffer::send, ktora jest powyzej :)
	}

	void synchronise(void)
	{
		// komunikacja wlasciwa
		ecp_command.instruction_type = lib::SYNCHRO;

		send(); // Wyslanie zlecenia synchronizacji
		query(); // Odebranie wyniku zlecenia

		synchronised = (reply_package.reply_type == lib::SYNCHRO_OK);
	}

	void send()
	{
		if (messip::port_send(EDP_fd, 0, 0, ecp_command, reply_package) == -1) {
			int e = errno; // kod bledu systemowego
			perror("ecp: Send to EDP_MASTER error");
			sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "ecp: Send to EDP_MASTER error");
			BOOST_THROW_EXCEPTION(exception::se_r());
		}
	}

	/**
	 * @brief checks robot reply_package and detects edp_error
	 * @return edp_error occurred
	 */
	bool is_EDP_error() const
	{
		if (reply_package.error_no.error0 || reply_package.error_no.error1) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * @brief finalizes the robot command in data port variant
	 */
	void finalize_data_port_command()
	{
		communicate_with_edp = true;

		if (is_new_data && is_new_request) {
			ecp_command.instruction_type = lib::SET_GET;
		} else if (is_new_data) {
			ecp_command.instruction_type = lib::SET;
		} else if (is_new_request) {
			ecp_command.instruction_type = lib::GET;
		} else {
			communicate_with_edp = false;
		}

		if (is_new_request) {
			ecp_command.get_type = ARM_DEFINITION;
		}
	}

};

typedef _ecp_robot <> ecp_robot;

} // namespace robot
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_ROBOT_H */
