/*
 * plan_iface.cc
 *
 *  Created on: Jan 20, 2012
 *      Author: ptroja
 */

#include <memory>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/version.hpp>

#include "plan.hxx"

std::auto_ptr <Plan> readPlanFromFile(const std::string & path)
{
	// Assume, that the XSD file is installed in the binary folder
	boost::filesystem::path xsdpath = boost::filesystem::current_path();
	xsdpath /= "plan.xsd";

	// XML validation settings
	xml_schema::Properties props;

	// Add XSD validation to parser's properties.
#if BOOST_VERSION >=104400
	props.no_namespace_schema_location (xsdpath.string());
#else
	props.no_namespace_schema_location(xsdpath.file_string());
#endif

	// Read plan from XML file
	try {
		// If we had no XML schema, then only limited validation is possible:
		// p = plan(path, xml_schema::Flags::dont_validate);

		// Parse file with all the schema checks
		return plan(path, 0, props);
	} catch (const xml_schema::Exception & e) {
		// Display detailed diagnostics
		std::cerr << e << std::endl;

		// And leave the rest to the high-level handler.
		throw;
	}
}

