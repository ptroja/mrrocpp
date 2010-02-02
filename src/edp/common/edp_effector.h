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
	 * \brief descriptor of ECP process sending a command.
	 *
	 * It is stored for a further reply purpose.
	 */
    int caller;				// by 7&Y

public:
    lib::configurator &config;
    lib::sr_edp *msg;
    lib::sr_edp *sh_msg;

    bool check_config(const std::string & s);
    bool initialize_communication (void);


#if !defined(USE_MESSIP_SRR)
    name_attach_t *attach;
#else /* USE_MESSIP_SRR */
    messip_channel_t *attach;
#endif /* USE_MESSIP_SRR */

    effector (lib::configurator &_config, lib::robot_name_t l_robot_name);
    virtual ~effector();

    int test_mode;

    // oczekuje na polecenie od ECP, wczytuje je,
    // okresla typ nadeslanej instrukcji
    lib::INSTRUCTION_TYPE receive_instruction (void); // by YW

    // wyslanie adekwatnej odpowiedzi do ECP
    void reply_to_instruction (void);

    void insert_reply_type (lib::REPLY_TYPE rt);

    virtual void main_loop(); // main loop
    virtual void create_threads () = 0;

    bool is_reply_type_ERROR() const;

    void establish_error (uint64_t err0, uint64_t err1);

    lib::REPLY_TYPE is_reply_type (void) const;

    uint64_t is_error_no_0 (void) const;
    uint64_t is_error_no_1 (void) const;

    // bufory:
    // - polecen przysylanych z ECP
    // - polecen przysylanych z ECP dla watku trans_t
    lib::ecp_command_buffer new_ecp_command;
    lib::c_buffer new_instruction, current_instruction;

    const lib::robot_name_t robot_name;

    lib::POSE_SPECIFICATION previous_set_arm_type; // by Y poprzedni sposob zadawania pozycji
};
/************************ EDP_EFFECTOR ****************************/


// Zwrocenie stworzonego obiektu - efektora. Funkcja implementowana w plikach efektorow konkretnych (jadro).
effector* return_created_efector (lib::configurator &_config);

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
