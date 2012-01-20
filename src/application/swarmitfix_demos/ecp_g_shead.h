/*
 *
 *Author: yoyek
 */

#ifndef ECP_G_SHEAD_SWARM_DEMO_SINGLE_AGENT_H_
#define ECP_G_SHEAD_SWARM_DEMO_SINGLE_AGENT_H_

#include "robot/shead/ecp_r_shead.h"
#include "robot/shead/dp_shead.h"

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

/*!
 * @brief generator to send the command prepared by MP to EDP shead motors
 * it waits for the command execution finish
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class rotation_command : public common::generator::_generator <ecp::shead::robot>
{
private:

	lib::epos::epos_simple_command mp_ecp_epos_simple_command;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	rotation_command(task_t & _ecp_task);

	bool first_step();
	bool next_step();

	void create_ecp_mp_reply();
	void get_mp_ecp_command();
};

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif
