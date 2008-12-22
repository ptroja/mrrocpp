#if !defined(_ECP_T_PAWEL_H)
#define _ECP_T_PAWEL_H

#include <sys/iofunc.h>
#include <sys/dispatch.h>

#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_pawel.h"
// Konfigurator.
#include "lib/configurator.h"

class ecp_task_pawel: public ecp_task  {
	protected:
		//	ecp_tff_nose_run_generator* nrg;
		//	ecp_teach_in_generator* tig;
		pawel_generator* pg;
		double x,y,z;


	public:
		// KONSTRUKTORY
		ecp_task_pawel(configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void task_initialization(void);
		void main_task_algorithm(void);
};

#endif
