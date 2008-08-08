#if !defined(_ECP_T_TB_IRP6OT_H)
#define _ECP_T_TB_IRP6OT_H

#include "ecp/common/ecp_task.h"

class ecp_t_tb_irp6ot: public ecp_task{
	public:
		ecp_t_tb_irp6ot(configurator &_config);
		~ecp_t_tb_irp6ot();
		
		void task_initialization(void);
		void main_task_algorithm(void);
};

#endif
