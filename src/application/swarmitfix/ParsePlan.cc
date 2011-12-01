/*
 * ParsePlan.cc
 *
 *  Created on: Nov 9, 2011
 *      Author: ptroja
 */

#include <iostream>
#include <memory>
#include <exception>

#include <boost/thread/thread.hpp>

#include "planner.h"
#include "base/lib/mrmath/homog_matrix.h"
#include "base/lib/mrmath/mrmath.h"

using namespace std;

int main(int argc, char *argv[])
{
	// Check for input arguments
	if(argc < 2) {
		cerr << "Usage: " << argv[0] << " plan_file.xml" << endl;
		return -1;
	}

	try {
		const Plan p = *plan(argv[1], xml_schema::Flags::dont_validate);

		// cerr << p.hNum() << endl;

		for(Plan::PkmType::ItemConstIterator it = p.pkm().item().begin();
				it != p.pkm().item().end();
				++it) {

			const Plan::PkmType::ItemType & pkmCmd = *it;

			// Test only 1st agent
			if(pkmCmd.agent() != 1)
				continue;

			using namespace mrrocpp;

			// Goal pose
			lib::Homog_matrix hm;

			if(pkmCmd.pkmToWrist().present()) {
				hm = lib::Homog_matrix(pkmCmd.pkmToWrist().get());
			} else if (pkmCmd.Xyz_Euler_Zyz().present()) {
				hm = lib::Xyz_Euler_Zyz_vector(
						pkmCmd.Xyz_Euler_Zyz()->x(),
						pkmCmd.Xyz_Euler_Zyz()->y(),
						pkmCmd.Xyz_Euler_Zyz()->z(),
						pkmCmd.Xyz_Euler_Zyz()->ox(),
						pkmCmd.Xyz_Euler_Zyz()->oy(),
						pkmCmd.Xyz_Euler_Zyz()->oz()
						);
			} else {
				// This should be already checked by XML validation
				throw std::runtime_error("Goal pose not defined");
			}

			std::cerr << pkmCmd.pkmToWrist() << std::endl;
			std::cerr << hm << std::endl << std::endl;
			//std::cerr << "[" << pkmCmd.l1() << "," << pkmCmd.l2() << "," << pkmCmd.l3() << "]" << std::endl;
		}

		// Create planner object
		//planner pp(argv[1]);

		// Start execution
		//pp.start();

	} catch (const xml_schema::Exception& e) {
		cerr << e << endl;
		return 1;
	}

	return 0;
}
