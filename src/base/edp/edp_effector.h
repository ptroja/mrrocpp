/*!
 * \file edp_effector.h
 * \brief File containing the declaration of edp::common::effector class.
 *
 * \author yoyek
 *
 */

#ifndef __EDP_EFFECTOR_H
#define __EDP_EFFECTOR_H

#include <boost/utility.hpp>

#include "edp_shell.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace common {

/*!
 * \class effector
 * \brief Base class of all EDP effectors.
 *
 * It implements mainly inter-process communication and EDP configuration management
 *
 * \author yoyek
 */
class effector : private boost::noncopyable
{
protected:
	/*!
	 * \brief real reply type of EDP process send to ECP process.
	 *
	 * It is used because reply_type can be temporarily changed while ECP command is interpreted
	 */
	lib::REPLY_TYPE real_reply_type;

	/*!
	 * \brief id of ECP process sending a command.
	 *
	 * It is stored for a further reply purpose.
	 */
	int caller;

	/*!
	 * \brief descriptor of EDP server attach point
	 *
	 * It is stored for a communication with ECP purpose.
	 */
	lib::fd_server_t server_attach;

	/*!
	 * \brief method to receive instruction from ECP
	 *
	 * IT also makes initial ECP command interpretation.
	 */
	template <typename ROBOT_COMMAND_T>
	lib::INSTRUCTION_TYPE receive_instruction(ROBOT_COMMAND_T & instruction_)
	{
		// oczekuje na polecenie od ECP, wczytuje je oraz zwraca jego typ
		int rcvid;
		/* Oczekiwanie na polecenie od ECP */

		// bufory:
		// - polecen przysylanych z ECP
		// - polecen przysylanych z ECP dla watku trans_t
		ROBOT_COMMAND_T new_ecp_command;

		/* Do your MsgReceive's here now with the chid */
		while (true) {

			int32_t type, subtype;
			rcvid = messip::port_receive(server_attach, type, subtype, new_ecp_command);

			if (rcvid == -1) {/* Error condition, exit */
				perror("messip::port_receive()");
				break;
			} else if (rcvid < -1) {
				fprintf(stderr, "ie. MESSIP_MSG_DISCONNECT\n");
				continue;
			}

			/* A message (presumable ours) received, handle */
			break;
		}

		caller = rcvid;

		instruction_ = new_ecp_command;

		//	if ((instruction.instruction_type == lib::SET) || (instruction.instruction_type == lib::SET_GET)) {
		//	std::cout << "edp effector: " << instruction.instruction_type << "\n";
		//	}

		return instruction_.instruction_type;
	}

	/*!
	 * \brief method to reply to ECP
	 *
	 * Basing on the previous computation.
	 */
	template <typename ROBOT_REPLY_T>
	void reply_to_instruction(ROBOT_REPLY_T & reply)
	{
		// Wyslanie potwierdzenia przyjecia polecenia do wykonania,
		// adekwatnej odpowiedzi na zapytanie lub
		// informacji o tym, ze przyslane polecenie nie moze byc przyjte
		// do wykonania w aktualnym stanie EDP

		if (!((reply.reply_type == lib::ERROR) || (reply.reply_type == lib::SYNCHRO_OK)))
			reply.reply_type = real_reply_type;

		if (messip::port_reply(server_attach, caller, 0, reply) == -1) {

			uint64_t e = errno;
			perror("Reply() to ECP failed");
			msg->message(lib::SYSTEM_ERROR, e, "Reply() to ECP failed");
			throw exception::se();
		}
		real_reply_type = lib::ACKNOWLEDGE;
	}

	/*!
	 * \brief method to establish error sent to ECP.
	 *
	 * The error is stored in reply_buffer.
	 */
	void establish_error(lib::r_buffer_base & reply, uint64_t err0, uint64_t err1);

public:

	/*!
	 * \brief EDP shell
	 */
	shell &edp_shell;

	/*!
	 * \brief Name of the robot
	 *
	 * For the identification purpose
	 */
	const lib::robot_name_t robot_name;

	/*!
	 * \brief Reference to configuration object
	 *
	 * It stores data read from ini file.
	 */
	lib::configurator &config;

	/*!
	 * \brief Pointer to object to communicate with UI SR thread outside the signal handlers.
	 *
	 * For the usage in asynchronous communication.
	 */
	boost::shared_ptr <lib::sr_edp> msg;

	/*!
	 * \brief Info if the robot test mode is active.
	 *
	 * It is taken from configuration data.
	 */
	bool robot_test_mode;

	/*!
	 * \brief Method to initiate communication.
	 *
	 * It opens the communication channels of EDP server.
	 * TODO: this should be void and throw an exception in case of failure
	 */
	void initialize_communication(void);

	/*!
	 * \brief Constructor.
	 *
	 * It connects to the existing channels of UI SR.
	 */
	effector(shell &_shell, const lib::robot_name_t & l_robot_name);

	/*!
	 * \brief Destructor.
	 *
	 * It destroys the objects to communicate with UI SR.
	 */
	virtual ~effector();

	/*!
	 * \brief Pure virtual method (main loop of the the process) to be implemented in child classes.
	 *
	 * Typically it is finite state automaton.
	 */
	virtual void main_loop() = 0; // main loop

	/*!
	 * \brief Pure virtual method to create effector specific threads.
	 *
	 * For the purpose of visualization, measurement registration, force control, etc.
	 */
	virtual void create_threads() = 0;

};
/************************ EDP_EFFECTOR ****************************/

/*!
 * \brief Method to return object of specific effector type.
 *
 * It is implemented in specific effector file.
 */
effector* return_created_efector(common::shell &_shell);

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
