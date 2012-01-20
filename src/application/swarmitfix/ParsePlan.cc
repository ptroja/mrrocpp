/*
 * ParsePlan.cc
 *
 *  Created on: Nov 9, 2011
 *      Author: ptroja
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <exception>

#include "planner.h"
#include "base/lib/mrmath/homog_matrix.h"
#include "base/lib/mrmath/mrmath.h"

#include "plan.hxx"

int main(int argc, char *argv[])
{
	// Check for input arguments
	if(argc < 2) {
		std::cerr << "Usage: " << argv[0] << " plan_file.xml" << std::endl;
		return -1;
	}

	try {
		// XML validation settings
		xml_schema::Properties props;

		// Add XSD validation to parser's properties
		props.no_namespace_schema_location ("plan.xsd");

		//const Plan p = *plan(argv[1], xml_schema::Flags::dont_validate);
		Plan p = *plan(argv[1], 0, props);

		std::cerr << "mbase item # " << p.mbase().item().size() << std::endl;
		std::cerr << "pkm item # " << p.pkm().item().size() << std::endl;

		std::ostringstream ostr;
		boost::archive::text_oarchive oa(ostr);
		xml_schema::ostream<boost::archive::text_oarchive> os (oa);

		os << p.pkm().item().front();

		// Print the text representation.
		//
		std::string str (ostr.str ());

		std::cerr << std::endl
			 << "text representation: " << std::endl
			 << str << std::endl;

		// Load from a text archive.
		//
		std::istringstream istr (str);
		boost::archive::text_iarchive ia (istr);
		xml_schema::istream<boost::archive::text_iarchive> is (ia);

		std::auto_ptr<Pkm::ItemType> copy (new Pkm::ItemType (is));

		std::cerr << *copy << std::endl;

		// Create planner object
		//planner pp(argv[1]);

		// Start execution
		//pp.start();

	} catch (const xml_schema::Exception & e) {
		std::cerr << "Exception::what(): " << e << std::endl;
		return 1;
	}

	return 0;
}
