// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/spkm/ecp_r_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	epos_command_data_port(EPOS_COMMAND_DATA_PORT),
			epos_reply_data_request_port(EPOS_REPLY_DATA_REQUEST_PORT),
			ecp_robot(lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION,
					_config, _sr_ecp), kinematics_manager() {
	add_data_ports();
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

}

robot::robot(common::task::task& _ecp_object) :
	epos_command_data_port(EPOS_COMMAND_DATA_PORT),
			epos_reply_data_request_port(EPOS_REPLY_DATA_REQUEST_PORT),
			ecp_robot(lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION,
					_ecp_object), kinematics_manager() {
	add_data_ports();
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

void robot::add_data_ports() {
	port_manager.add_port(&epos_command_data_port);
	port_manager.add_port(&epos_reply_data_request_port);
}

void robot::create_command() {

	if (epos_command_data_port.is_new_data()) {
		epos_data_port_command_structure = epos_command_data_port.get();
		// generator command interpretation
		// narazie proste przepisanie
		for (int i = 0; i < 6; i++) {
			ecp_edp_cbuffer.em[i] = epos_data_port_command_structure.em[i];
			ecp_edp_cbuffer.emdm[i] = epos_data_port_command_structure.emdm[i];
			ecp_edp_cbuffer.aa[i] = epos_data_port_command_structure.aa[i];
			ecp_edp_cbuffer.da[i] = epos_data_port_command_structure.da[i];
			ecp_edp_cbuffer.av[i] = epos_data_port_command_structure.av[i];
		}
		ecp_edp_cbuffer.tt = epos_data_port_command_structure.tt;
		ecp_edp_cbuffer.profile_type
				= epos_data_port_command_structure.profile_type;
	} else {
		ecp_edp_cbuffer.profile_type = lib::EPOS_GEN_PROFILE_NO_ACTION;
	}
	// message serialization
	memcpy(ecp_command.instruction.arm.serialized_command, &ecp_edp_cbuffer,
			sizeof(ecp_edp_cbuffer));
}

void robot::get_reply() {
	// message deserialization
	memcpy(&edp_ecp_rbuffer, reply_package.arm.serialized_reply,
			sizeof(edp_ecp_rbuffer));

	// generator reply generation
	for (int i = 0; i < 6; i++) {
		epos_data_port_reply_structure.position[i]
				= edp_ecp_rbuffer.position[i];
		epos_data_port_reply_structure.motion_in_progress[i]
				= edp_ecp_rbuffer.motion_in_progress[i];
	}
	if (epos_reply_data_request_port.is_new_request()) {
		epos_reply_data_request_port.set(epos_data_port_reply_structure);
	}
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void) {
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

