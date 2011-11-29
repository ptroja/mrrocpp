/*
 * ParsePlan.cc
 *
 *  Created on: Nov 9, 2011
 *      Author: ptroja
 */

#include <iostream>
#include <memory>

#include <boost/thread/thread.hpp>

#include "planner.h"
#include "base/lib/mrmath/homog_matrix.h"

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

		cerr << p.hNum() << endl;

		for(Plan::PkmType::ItemConstIterator it = p.pkm().item().begin();
				it != p.pkm().item().end();
				++it) {

			const Plan::PkmType::ItemType & pkmCmd = *it;

			// Test only 1st agent
			if(pkmCmd.agent() != 1)
				continue;

			mrrocpp::lib::Homog_matrix hm(pkmCmd.pkmToWrist());

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
