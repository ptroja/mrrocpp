#if !defined(_ECP_T_PW_SCENA_IRP6OT_H)
#define _ECP_T_PW_SCENA_IRP6OT_H



#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "ecp/common/ecp_task.h"


#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_pw_scena.h"
#include "ecp/common/ecp_g_smooth.h"

#include "ecp_mp/ecp_mp_s_cvfradia.h"

class ecp_task_pw_scena_irp6ot: public ecp_task  {

	ecp_g_pw_scena* scena_gen;
	//Smoth movement generator
	ecp_smooth_generator* smooth_gen;

public:
	//Konstruktory.
	ecp_task_pw_scena_irp6ot(configurator &_config);

	//Methods for ECP template to redefine in concrete classes.
	void task_initialization(void);
	void main_task_algorithm(void);
};

#endif
