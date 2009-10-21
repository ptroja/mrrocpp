// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_EFFECTOR_H
#define __EDP_EFFECTOR_H

#include <stdint.h>
#if !defined(USE_MESSIP_SRR)
#include <sys/dispatch.h>
#else
#include "lib/messip/messip.h"
#endif /* !USE_MESSIP_SRR */
#include <string>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"

#include "kinematics/common/transformer_error.h"
#include "kinematics/common/kinematics_manager.h"
#include "edp/common/edp.h"


// Konfigurator
#include "lib/configurator.h"


namespace mrrocpp {
namespace edp {
namespace common {


// Glowna klasa efektora EDP
class effector : public kinematic::common::transformer_error
{
protected:

    // faktyczny typ odpowiedzi dla ECP
    // (przechowuje typ odpowiedzi, gdy reply_type jest chwilowo zmienione)
    lib::REPLY_TYPE real_reply_type;

    // bufor odpowiedzi wysylanych do ECP/MP
    lib::r_buffer reply;

    int caller;				// by 7&Y

public:
    lib::configurator &config;
    lib::sr_edp *msg;

    bool check_config(const std::string & s);
    bool initialize_communication (void);

    std::string mrrocpp_network_path;

#if !defined(USE_MESSIP_SRR)
    name_attach_t *attach;
#else /* USE_MESSIP_SRR */
    messip_channel_t *attach;
#endif /* USE_MESSIP_SRR */

    effector (lib::configurator &_config, lib::ROBOT_ENUM l_robot_name);
    virtual ~effector();

    lib::controller_state_t controller_state_edp_buf; // do okreslenia stanu robota

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

    const lib::ROBOT_ENUM robot_name;

    lib::POSE_SPECIFICATION previous_set_arm_type; // by Y poprzedni sposob zadawania pozycji
};
/************************ EDP_EFFECTOR ****************************/


// Zwrocenie stworzonego obiektu - efektora. Funkcja implementowana w plikach efektorow konkretnych (jadro).
effector* return_created_efector (lib::configurator &_config);

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
