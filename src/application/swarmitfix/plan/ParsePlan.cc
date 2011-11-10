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
	try {
		const Plan_t p = *plan("PlanXML.xml", xml_schema::flags::dont_validate);

		cerr << p.hNum() << endl;
	} catch (const xml_schema::exception& e) {
		cerr << e << endl;
		return 1;
	}

	return 0;
}
