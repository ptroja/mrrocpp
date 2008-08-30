#if !defined(_ECP_T_TB_IRP6OT_H)
#define _ECP_T_TB_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

class ecp_t_tb_irp6ot: public ecp_task
{
	protected:
		//smoth movement generator
		ecp_smooth_generator* sgen;
		//calibration of force
		bias_edp_force_generator* befgen;
		//gripper approach with force control
		ecp_tff_gripper_approach_generator* gagen;

	public:
		ecp_t_tb_irp6ot(configurator &_config);
		~ecp_t_tb_irp6ot();

		void task_initialization(void);
		void main_task_algorithm(void);
};

#endif

