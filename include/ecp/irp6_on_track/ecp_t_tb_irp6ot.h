#if !defined(_ECP_T_TB_IRP6OT_H)
#define _ECP_T_TB_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "lib/com_buf.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_g_sleep.h"
#include "ecp_mp/ecp_mp_s_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class tb: public common::task::task{
	protected:
		common::generator::smooth* sgen;					//smooth movement generator
		common::generator::bias_edp_force* befgen;			//calibration of force
		common::generator::tff_gripper_approach* gagen;	//gripper approach with force control
		common::generator::linear *lgen;					//linear generator
		lib::trajectory_description tdes;				//trajectory description from com_buf.h
		common::task::ecp_sub_task_gripper_opening* go_st;		//sub_task_gripper_opening
		common::generator::sleep* sleepgen;				//sleep generator

	public:
		tb(lib::configurator &_config);
		~tb();
		void set_tdes(double, double, double, double, double, double, double);
		void init_tdes(lib::POSE_SPECIFICATION, int);
		void task_initialization(void);
		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

