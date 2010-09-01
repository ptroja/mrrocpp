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
#include "base/lib/srlib.h"
#include "base/lib/configurator.h"
#include "base/ecp_mp/ecp_mp_robot.h"
#include "base/lib/single_thread_port.h"

#if defined(USE_MESSIP_SRR)
#include "messip.h"
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
	 * used e.g. in transparent generator in strict coordination
	 * @param[in] mp_buffer buffer including mp command
	 */
	void copy_mp_to_edp_buffer(const lib::c_buffer& mp_buffer);

	/**
	 * @brief method to directly copy edp reply to mp reply
	 * used e.g. in transparent generator in strict coordination
	 * @param[out] mp_buffer buffer including mp reply
	 */
	void copy_edp_to_mp_buffer(lib::r_buffer& mp_buffer);

	/**
	 * @brief method to spawn and connect to EDP
	 * when called from Ui it first spawns then connects to EDP,
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
	 * if the flag is set the EDP is spawned with robot object creation then killed with destruction;
	 * it is set when UI calls robot constructor
	 */
	const bool spawn_and_kill;

protected:

public:
	// to exchange data with generators
	lib::single_thread_port_manager port_manager;

	/**
	 * @brief the communication with EDP flag
	 * if the flag is set (default) the ECP communicates with EDP in Move method of generator
	 * Sometimes it is needed to disable communication e.g. when there is a need to communicate only With MP or VSP
	 * in the following iterations of Move
	 */
	bool communicate_with_edp;

	virtual void create_command();
	virtual void get_reply();

	//! Wyslanie do EDP polecenia
	void send();
	void query();

	lib::ecp_command_buffer ecp_command;
	lib::r_buffer reply_package;

	lib::sr_ecp & sr_ecp_msg; // obiekt do komunikacji z SR

	bool synchronised; // Flaga synchronizacji robota (true - zsynchronizowany, false - nie)

	int number_of_servos;
	const std::string edp_section;

#if !defined(USE_MESSIP_SRR)
	int EDP_fd; // by Y&W
#else
	messip_channel_t *EDP_fd;
#endif

	virtual void execute_motion(void);
	// Zlecenie wykonania ruchu przez robota (realizowane przez klase konkretna):
	// na poziomie ECP jest to polecenie dla EDP

			ecp_robot(lib::robot_name_t _robot_name, int _number_of_servos, const std::string &_edp_section, lib::configurator &_config, lib::sr_ecp &_sr_ecp);
			ecp_robot(lib::robot_name_t _robot_name, int _number_of_servos, const std::string &_edp_section, common::task::task& _ecp_object);

	pid_t get_EDP_pid(void) const;

	// destruktor by Y - do usuniecia obiektu do komunikacji z SR
	virtual ~ecp_robot(void);

	//! Zlecenie synchronizacji robota
	void synchronise(void);

	// Pobranie aktualnych polozen

	bool is_synchronised(void) const; // Czy robot zsynchronizowany?

};

class ECP_error
{ // Klasa obslugi bledow robota
public:
	const lib::error_class_t error_class;
	const uint64_t error_no;
	lib::edp_error error;

	ECP_error(lib::error_class_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
};

class ECP_main_error
{ // Klasa obslugi bledow ECP
public:
	const lib::error_class_t error_class;
	const uint64_t error_no;

	ECP_main_error(lib::error_class_t err_cl, uint64_t err_no);
};

} // namespace robot
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_ROBOT_H */
