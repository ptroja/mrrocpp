/*
 * mp_t_swarmitfix_logic.cc
 *
 *  Created on: Nov 23, 2011
 *      Author: ptroja
 */

#include "mp_t_swarmitfix.h"

#include "robot/spkm/const_spkm1.h"
#include "robot/spkm/const_spkm2.h"
#include "robot/smb/const_smb1.h"
#include "robot/smb/const_smb2.h"

#include "base/lib/swarmtypes.h"

namespace mrrocpp {
namespace mp {
namespace task {

bool swarmitfix::b1_initial_condition(void) const
{
	return (current_plan_status == ONGOING);
}

bool swarmitfix::b1_terminal_condition(void) const
{
	return (current_plan_status == FAILURE);
}

void swarmitfix::b1_handle_spkm1_notification()
{
	if (IO.transmitters.spkm1.inputs.notification->access == lib::ACK) {
		current_workers_status[lib::spkm1::ROBOT_NAME] = IDLE;
	} else {
		current_plan_status = FAILURE;
	}
}


void swarmitfix::b1_handle_spkm2_notification()
{
	if (IO.transmitters.spkm2.inputs.notification->access == lib::ACK) {
		current_workers_status[lib::spkm2::ROBOT_NAME] = IDLE;
	} else {
		current_plan_status = FAILURE;
	}
}

void swarmitfix::b1_handle_smb1_notification()
{
	if (IO.transmitters.smb1.inputs.notification->access == lib::ACK) {
		current_workers_status[lib::smb1::ROBOT_NAME] = IDLE;
	} else {
		current_plan_status = FAILURE;
	}
}

void swarmitfix::b1_handle_smb2_notification()
{
	if (IO.transmitters.smb2.inputs.notification->access == lib::ACK) {
		current_workers_status[lib::smb2::ROBOT_NAME] = IDLE;
	} else {
		current_plan_status = FAILURE;
	}
}

void swarmitfix::b1_plan_progress()
{
	// TODO
}

bool swarmitfix::b2_initial_condition(void) const
{
	return (current_plan_status == FAILURE);
}

bool swarmitfix::b2_terminal_condition(void) const
{
	return false;
}

void swarmitfix::b2_stop_all()
{
	lib::spkm::next_state_t spkm_stop_command;

	spkm_stop_command.variant = lib::spkm::STOP;

	if(current_workers_status[lib::spkm1::ROBOT_NAME] == BUSY) {
		IO.transmitters.spkm1.outputs.command->Send(spkm_stop_command);
	}
	if(current_workers_status[lib::spkm2::ROBOT_NAME] == BUSY) {
		IO.transmitters.spkm2.outputs.command->Send(spkm_stop_command);
	}
	if(current_workers_status[lib::smb1::ROBOT_NAME] == BUSY) {
		// TODO
	}
	if(current_workers_status[lib::smb2::ROBOT_NAME] == BUSY) {
		// TODO
	}
}

}
}
}
