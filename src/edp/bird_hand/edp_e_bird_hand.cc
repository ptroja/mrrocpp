// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6m_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6m_effector
//				- definicja funkcji return_created_efector()
//
// Autor:
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp/bird_hand/edp_e_bird_hand.h"
#include "edp/common/reader.h"
// Kinematyki.
#include "kinematics/bird_hand/kinematic_model_bird_hand.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/vis_server.h"
#include "lib/epos_gen.h"

#define PORT "/dev/ser2"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace bird_hand {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb) {
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::get_controller_state(lib::c_buffer &instruction) {


	for(uint8_t i=0; i<8; i++)
	{
		int16_t abspos;
		if(i<1)
			device.getSynchroPos(i, abspos);
		synchro_position[i] = (int32_t)(abspos * 124345.23443); // TODO : tu wspawiæ magiczn¹ sta³¹
	}

	printf("synchro position readed : %d \n", synchro_position[0]);

	controller_state_edp_buf.is_synchronised = true;

	reply.controller_state = controller_state_edp_buf;

	Eigen::Matrix<int32_t, 8, 1> pos_tmp;

	for(uint8_t i=0; i<8; i++)
	{
		int32_t pos;
		int16_t t, c;
		uint8_t status;
		if(i<1)
			device.getStatus(i, status, pos, c, t);

		pos_tmp[i] = pos;
	}

	pos_tmp += synchro_position;

	printf("position readed : %d \n", pos_tmp[0]);

	fflush(stdout);

	current_motor_pos = pos_tmp.cast<double>();

	// dla pierwszego wypelnienia current_joints
	get_current_kinematic_model()->mp2i_transform(current_motor_pos,
			current_joints);

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for (int i = 0; i < number_of_servos; i++) {
			servo_current_motor_pos[i] = desired_motor_pos_new[i]
					= desired_motor_pos_old[i] = current_motor_pos[i];
			desired_joints[i] = current_joints[i];
		}
	}
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::ROBOT_BIRD_HAND) {
	number_of_servos = BIRD_HAND_NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();

	device.connect(PORT);
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction) {
	msg->message("move_arm");
	lib::bird_hand_cbuffer ecp_edp_cbuffer;
	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command,
			sizeof(ecp_edp_cbuffer));

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	switch (ecp_edp_cbuffer.variant) {
	case lib::BIRD_HAND_CBUFFER_EPOS_GEN_PARAMETERS: {
		// epos parameters computation basing on trajectory parameters
		lib::epos_gen_parameters epos_gen_parameters_structure;
		lib::epos_low_level_command epos_low_level_command_structure;

		memcpy(&epos_gen_parameters_structure,
				&(ecp_edp_cbuffer.epos_gen_parameters_structure),
				sizeof(epos_gen_parameters_structure));

		compute_epos_command(epos_gen_parameters_structure,
				epos_low_level_command_structure);

		ss << ecp_edp_cbuffer.epos_gen_parameters_structure.dm[4];

		msg->message(ss.str().c_str());

		// previously computed parameters send to epos2 controllers


		// start the trajectory execution

	}
		break;
	case lib::BIRD_HAND_CBUFFER_EPOS_LOW_LEVEL_COMMAND: {
		lib::epos_low_level_command epos_low_level_command_structure;
		memcpy(&epos_low_level_command_structure,
				&(ecp_edp_cbuffer.epos_low_level_command_structure),
				sizeof(epos_low_level_command_structure));

	}
		break;
	default:
		break;

	}

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction) {

	Eigen::Matrix<int32_t, 8, 1> pos_tmp;

	for(uint8_t i=0; i<8; i++)
	{
		int32_t pos;
		int16_t t, c;
		uint8_t status;

		device.getStatus(i, status, pos, c, t);

		pos_tmp[i] = pos;
	}

	pos_tmp -= synchro_position;


	//?????????????????????????????????????????????????????????????????????????????//

	//lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	//	printf(" GET ARM\n");
	//	flushall();
	static int licznikaaa = (-11);

	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	ss << "get_arm_position: " << licznikaaa;
	msg->message(ss.str().c_str());
	//	printf("%s\n", ss.str().c_str());

	lib::bird_hand_rbuffer edp_ecp_rbuffer;
	edp_ecp_rbuffer.epos_controller[3].position = licznikaaa;

	if (licznikaaa < 10) {
		for (int i = 0; i < 6; i++) {
			edp_ecp_rbuffer.epos_controller[i].motion_in_progress = true;
		}

	} else {
		for (int i = 0; i < 6; i++) {
			edp_ecp_rbuffer.epos_controller[i].motion_in_progress = false;
		}
	}
	licznikaaa++;
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer,
			sizeof(edp_ecp_rbuffer));

	reply.servo_step = step_counter;
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota.
void effector::create_kinematic_models_for_given_robot(void) {
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::bird_hand::kinematic_model_bird_hand());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
void effector::create_threads() {
	rb_obj = new common::reader_buffer(*this);
	vis_obj = new common::vis_server(*this);
}

}
// namespace bird_hand


namespace common {

// Stworzenie obiektu edp_bird_hand_effector.
effector* return_created_efector(lib::configurator &_config) {
	return new bird_hand::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

