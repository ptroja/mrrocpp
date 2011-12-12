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

#include "base/lib/periodic_timer.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

class quickstop : public common::generator::_generator<ecp::smb::robot>
{
public:
	//! Constructor
	quickstop(task_t & _ecp_task);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();
};

/*!
 * @brief generator to send the command prepared EDP SMB legs
 * @note waits for the command execution is finished
 * @ingroup generators
 */
class stand_up : public common::generator::_generator<ecp::smb::robot>
{
public:
	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference
	 * @cmd cmd command
	 */
	stand_up(task_t & _ecp_task, const lib::smb::festo_command_td & cmd = lib::smb::festo_command_td());

	bool first_step();

	bool next_step();

private:
	lib::smb::festo_command_td festo_command;
};

/*!
 * @brief generator to send the command to EDP SMB motors
 * @note waits for the command execution is finished
 * @ingroup generators
 */
class rotate : public common::generator::_generator<ecp::smb::robot>
{
public:
	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	rotate(task_t & _ecp_task, const lib::smb::smb_epos_simple_command & cmd);

	bool first_step();

	bool next_step();

private:
	lib::smb::smb_epos_simple_command simple_command;

	//! Periodic query timer
	lib::periodic_timer wakeup;
};


} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
