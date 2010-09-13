/*!
 * @file configsrv.cc
 * @brief Class for configuration server - definitions
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <iostream>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "base/lib/configsrv.h"

// Konstruktor obiektu - konfiguratora.
configsrv::configsrv (const std::string & _node, const std::string & _dir, const std::string & _ini_file)
	: node(_node), dir(_dir), ini_file(_ini_file)
{
	// Stworzenie sciezki do pliku.
	mrrocpp_network_path = "/net/";
	mrrocpp_network_path += node;
	mrrocpp_network_path += dir;

	// Stworzenie sciezki do pliku.
	file_location = mrrocpp_network_path;
	file_location += "configs/";
	file_location += ini_file;

	// Stworzenie sciezki do pliku.
	common_file_location = mrrocpp_network_path;
	common_file_location += "configs/";
	common_file_location += "common.ini";

	// Read configuration
	read_property_tree_from_file(file_pt, file_location);
	read_property_tree_from_file(common_file_pt, common_file_location);
}

void configsrv::read_property_tree_from_file(boost::property_tree::ptree & pt, const std::string & file)
{
	try {
		if(boost::filesystem::extension(file) == ".ini") {
			boost::property_tree::read_ini(file, pt);
		} else if (boost::filesystem::extension(file) == ".xml") {
			boost::property_tree::read_xml(file, pt);
		} else {
			throw std::logic_error("unknown config file extension");
		}
	} catch (boost::property_tree::ptree_error & e) {
		std::cerr << e.what() << std::endl;
	}
}

void configsrv::change_ini_file (const std::string & _ini_file)
{
	ini_file = _ini_file;

	// Reload configuration
	read_property_tree_from_file(file_pt, file_location);
	read_property_tree_from_file(common_file_pt, common_file_location);
}

bool configsrv::exists(const std::string & _key, const std::string & _section_name) const
{
	try {
		value<std::string>(_key, _section_name);
	} catch (boost::property_tree::ptree_bad_path & e) {
		return false;
	}

	return true;
}
