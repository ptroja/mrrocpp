/*
 * mp_t_pnml.cc
 *
 *  Created on: Oct 19, 2009
 *      Author: ptroja
 */

#include <iostream>

#include "mp_t_pnml.h"

namespace mrrocpp {
namespace mp {
namespace task {

pnmlAutomat::pnmlAutomat(lib::configurator &_config) :
	mrrocpp::mp::task::task(_config)
{
	std::string pnmlFile = _config.value<std::string>("pnml_file", "[mp]");
	std::cerr << "pnmlFile: " << pnmlFile << std::endl;

	try {
		pnmlNet.BuildFromPNML(pnmlFile.c_str());
	} catch (const xml_schema::exception& e) {
		std::cerr << e << std::endl;
		// TODO: throw something like SYSTEM_ERROR
	}

	executor = new generator::pnmlExecutor(*this, pnmlNet);
	executor->robot_m = this->robot_m;
}

pnmlAutomat::~pnmlAutomat() {
	delete executor;
}

void pnmlAutomat::main_task_algorithm(void)
{
	executor->Move();
}

task* return_created_mp_task(lib::configurator &_config)
{
	return new pnmlAutomat(_config);
}

}
}
}
