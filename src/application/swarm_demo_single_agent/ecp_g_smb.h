/*
 * generator/ecp_g_smb.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SMB_SWARM_DEMO_SINGLE_AGENT_H_
#define ECP_G_SMB_SWARM_DEMO_SINGLE_AGENT_H_

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
class legs_command : public common::generator::generator
{

private:
	lib::smb::festo_command_td mp_ecp_festo_command;

	/**
	 * @brief pin insertion command data port
	 */
	lib::single_thread_port <lib::smb::festo_command_td> *smb_festo_command_data_port;

	/**
	 * @brief leg status reply data request port
	 */
	lib::single_thread_request_port <lib::smb::multi_leg_reply_td> *smb_multi_leg_reply_data_request_port;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	legs_command(common::task::task& _ecp_task);

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
class external_epos_command : public common::generator::generator
{
private:

	lib::epos::epos_simple_command mp_ecp_epos_simple_command;

	/**
	 * @brief epos external motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_simple_command> *epos_external_command_data_port;

	/**
	 * @brief epos motion status with external reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> *epos_external_reply_data_request_port;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	external_epos_command(common::task::task& _ecp_task);

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
