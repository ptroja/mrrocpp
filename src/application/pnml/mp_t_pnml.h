/*
 * mp_t_pnml.h
 *
 *  Created on: Oct 19, 2009
 *      Author: ptroja
 */

#ifndef MP_T_PNML_H_
#define MP_T_PNML_H_

#include "mp/PNExec/Net.hh"

#include "mp/mp.h"
#include "mp/mp_task.h"
#include "mp/mp_g_pnml.h"

namespace mrrocpp {
namespace mp {
namespace task {

class pnmlAutomat : public mrrocpp::mp::task::task {
	public:
		// methods for mp template
		void main_task_algorithm(void);

		pnmlAutomat(lib::configurator &_config);
		~pnmlAutomat();

	private:
		pnexec::Net pnmlNet;
		generator::pnmlExecutor *executor;
};

}
}
}

#endif /* MP_T_PNML_H_ */
