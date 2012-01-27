/*
 * generator/ecp_g_smb.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SMB_SWARM_DEMO_SINGLE_AGENT_H_
#define ECP_G_SMB_SWARM_DEMO_SINGLE_AGENT_H_
#include "robot/smb/ecp_r_smb.h"
#include "base/ecp/ecp_generator.h"
#include "robot/smb/dp_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

/*!
 * @brief generator to send the command prepared by MP to EDP SMB legs
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class legs_command : public common::generator::_generator <ecp::smb::robot>
{

private:
	lib::smb::festo_command_td mp_ecp_festo_command;

public:
	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	legs_command(task_t & _ecp_task);

	bool first_step();
	bool next_step();

	void create_ecp_mp_reply();
	void get_mp_ecp_command();

};

/*!
 * @brief generator to send the command prepared by MP to EDP SMB motors
 * it waits for the command execution finish
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class external_epos_command : public common::generator::_generator <ecp::smb::robot>
{
private:
	lib::smb::motor_command mp_ecp_epos_simple_command;

public:
	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	external_epos_command(task_t & _ecp_task);

	bool first_step();
	bool next_step();

	void create_ecp_mp_reply();
	void get_mp_ecp_command();
};

} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
