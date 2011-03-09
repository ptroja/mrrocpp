/*
 * generator/ecp_g_epos.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SPKM_H_
#define ECP_G_SPKM_H_

#include "robot/spkm/ecp_r_spkm.h"

#include "base/ecp/ecp_generator.h"
#include "robot/epos/dp_epos.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace generator {

class spkm_pose : public common::generator::_generator<ecp::spkm::robot>
{
public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	spkm_pose(task_t & _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation
};

class spkm_quickstop : public common::generator::_generator<ecp::spkm::robot>
{
public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	spkm_quickstop(task_t & _ecp_task); //constructor

	bool first_step(); //first step generation
	bool next_step(); //next step generation
};

} // namespace generator
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

#endif
