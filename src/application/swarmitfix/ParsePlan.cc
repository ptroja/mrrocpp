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

using namespace std;

int main(int argc, char *argv[])
{
	// Check for input arguments
	if(argc < 2) {
		cerr << "Usage: " << argv[0] << " plan_file.xml" << endl;
		return -1;
	}

	try {
		// Create planner object
		planner pp(argv[1]);

		// Start execution
		pp.start();

	} catch (const xml_schema::Exception& e) {
		cerr << e << endl;
		return 1;
	}

	return 0;
}
