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

			// Goal pose
			mrrocpp::lib::Homog_matrix hm;

			if(pkmCmd.pkmToWrist().present()) {
				hm = mrrocpp::lib::Homog_matrix(pkmCmd.pkmToWrist().get());
			} else if (pkmCmd.Xyz_Angle_Axis().present()) {
				hm = mrrocpp::lib::Xyz_Angle_Axis_vector(
						pkmCmd.Xyz_Angle_Axis()->x(),
						pkmCmd.Xyz_Angle_Axis()->y(),
						pkmCmd.Xyz_Angle_Axis()->z(),
						pkmCmd.Xyz_Angle_Axis()->ax(),
						pkmCmd.Xyz_Angle_Axis()->ay(),
						pkmCmd.Xyz_Angle_Axis()->az()
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
