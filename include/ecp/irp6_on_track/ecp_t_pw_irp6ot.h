#if !defined(_ECP_T_PW_IRP6OT_H)
#define _ECP_T_PW_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "common/com_buf.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_g_sleep.h"

#include "ecp/irp6_on_track/ecp_g_pw_kolo.h"

class ecp_task_pw_irp6ot: public ecp_task  {

	ecp_g_pw_kolo * kolo_gen;

public:
	//Konstruktory.
	ecp_task_pw_irp6ot(configurator &_config);

	//Methods for ECP template to redefine in concrete classes.
	void task_initialization(void);
	void main_task_algorithm(void);
};

#endif
