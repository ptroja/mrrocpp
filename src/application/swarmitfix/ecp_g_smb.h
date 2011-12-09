/*
 * ecp_g_smb.h
 *
 * Author: ptroja
 */

#ifndef ECP_G_SMB_H_
#define ECP_G_SMB_H_

#include "robot/smb/ecp_r_smb.h"
#include "robot/smb/dp_smb.h"

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

class action_executor : public common::generator::_generator<ecp::smb::robot>
{
public:
	//! Constructor
	action_executor(task_t & _ecp_task, const lib::smb::next_state_t::action_sequence_t & _actions);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();

private:
	//! Motion action iterator
	lib::smb::next_state_t::action_sequence_t::const_iterator action_iterator;

	//! Request execution of a single motion action
	void request_action_execution(robot_t & robot, const lib::smb::action & action);

	//! Motion actions
	const lib::smb::next_state_t::action_sequence_t & actions;
};

class quickstop_executor : public common::generator::_generator<ecp::smb::robot>
{
public:
	//! Constructor
	quickstop_executor(task_t & _ecp_task);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();
};

} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
