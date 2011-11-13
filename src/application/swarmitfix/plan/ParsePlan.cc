/*
 * ParsePlan.cc
 *
 *  Created on: Nov 9, 2011
 *      Author: ptroja
 */

#include <iostream>
#include <memory>

#include "plan.hxx"

using namespace std;

int main(int argc, char *argv[])
{
	if(argc < 1) {
		cerr << "Usage: " << argv[0] << " plan_file.xml" << endl;
		return -1;
	}
	try {
		const Plan p = *plan(argv[1], xml_schema::Flags::dont_validate);

		cerr << p.hNum() << endl;
	} catch (const xml_schema::Exception& e) {
		cerr << e << endl;
		return 1;
	}

	return 0;
}
