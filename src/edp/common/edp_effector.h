/*!
 * \file edp_effector.h
 * \brief File containing the declaration of edp::common::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 *
 *  $LastChangedRevision: $
 *  $LastChangedDate:  $
 *  $LastChangedBy:  $
 *
 */

#ifndef __EDP_EFFECTOR_H
#define __EDP_EFFECTOR_H

#include <stdint.h>
#if !defined(USE_MESSIP_SRR)
#include <sys/dispatch.h>
#else
#include <messip.h>
#endif /* !USE_MESSIP_SRR */
#include <string>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "edp/common/edp.h"
#include "kinematics/common/kinematics_manager.h"

// Konfigurator
#include "lib/configurator.h"

#include "lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace common {

/*!
 * \class effector
 * \brief Base class of all EDP effectors.
 *
 * It implements mainly inter process communication and EDP configuration managment
 *
 * \author yoyek
 */
class effector
{
protected:

	/*!
	 * \brief real reply type of EDP process send to ECP process.
	 *
	 * It is used because reply_type can be temporarily changed while ECP command is interpreted
	 */
	lib::REPLY_TYPE real_reply_type;

	/*!
	 * \brief structure of reply of EDP process send to ECP process.
	 *
	 * It is used a union of structures for all EDP's
	 */
	lib::r_buffer reply;

	/*!
	 * \brief indentificator of ECP process sending a command.
	 *
	 * It is stored for a further reply purpose.
	 */
	int caller; // by 7&Y

#if !defined(USE_MESSIP_SRR)
	/*!
	 * \brief indentificator of EDP server attach point
	 *
	 * It is stored for a communication with ECP purpose.
	 */
	name_attach_t *server_attach;
#else /* USE_MESSIP_SRR */
	messip_channel_t *server_attach;
#endif /* USE_MESSIP_SRR */

	/*!
	 * \brief method to receive instruction from ECP
	 *
	 * IT also makes initial ECP command interpretation..
	 */
	lib::INSTRUCTION_TYPE receive_instruction(void); // by YW

	/*!
	 * \brief method to reply to ECP
	 *
	 * Basing on the previous computation..
	 */
	void reply_to_instruction(void);

	/*!
	 * \brief method to establish error sent to ECP.
	 *
	 * The error is stored in reply_buffer.
	 */
	void establish_error(uint64_t err0, uint64_t err1);

public:

	const lib::robot_name_t robot_name;
	lib::configurator &config;
	lib::sr_edp *msg;
	lib::sr_edp *sh_msg;

	int test_mode;

	bool initialize_communication(void);

	effector(lib::configurator &_config, lib::robot_name_t l_robot_name);
	virtual ~effector();

	lib::c_buffer instruction;

	virtual void main_loop() = 0; // main loop
	virtual void create_threads() = 0;

};
/************************ EDP_EFFECTOR ****************************/

// Zwrocenie stworzonego obiektu - efektora. Funkcja implementowana w plikach efektorow konkretnych (jadro).
effector* return_created_efector(lib::configurator &_config);

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
