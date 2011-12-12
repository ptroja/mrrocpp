/*
 * ParsePlan.cc
 *
 *  Created on: Nov 9, 2011
 *      Author: ptroja
 */

#include <iostream>
#include <memory>
#include <exception>

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
		// XML validation settings
		xml_schema::Properties props;

		// Add XSD validation to parser's properties
		props.no_namespace_schema_location ("plan.xsd");

		//const Plan p = *plan(argv[1], xml_schema::Flags::dont_validate);
		const Plan p = *plan(argv[1], 0, props);

		cerr << "head item # " << p.head().item().size() << endl;
		cerr << "mbase item # " << p.mbase().item().size() << endl;
		cerr << "pkm item # " << p.pkm().item().size() << endl;

		// Create planner object
		//planner pp(argv[1]);

		// Start execution
		//pp.start();

	} catch (const xml_schema::Exception & e) {
		cerr << "Exception::what(): " << e << endl;
		return 1;
	}

	return 0;
}
