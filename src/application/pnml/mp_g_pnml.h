/*
 * mp_g_pnml.h
 *
 *  Created on: Oct 20, 2009
 *      Author: ptroja
 */

#ifndef MP_G_PNML_H_
#define MP_G_PNML_H_

#include "base/mp/generator/mp_generator.h"
#include "PNExec/Net.hh"

namespace mrrocpp {
namespace mp {
namespace generator {

class pnmlExecutor : public generator
{
public:
	pnmlExecutor(task::task & _mp_task, pnexec::Net & _net);

	bool first_step();

	bool next_step();

private:
	pnexec::Net & pnmlNet;
};

}
}
}

#endif /* MP_G_PNML_H_ */
