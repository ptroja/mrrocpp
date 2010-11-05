/*!
 * @file configsrv.h
 * @brief Class for configuration server - declarations
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#if !defined(_CONFIGSRV_H)
#define _CONFIGSRV_H

#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

class configsrv
{
private:
	const std::string dir;
	std::string ini_file, mrrocpp_network_path;
	std::string file_location, common_file_location;

	/**
	 * Read property tree from configuration file
	 * @param pt property tree
	 * @param file configuration file
	 */
	void read_property_tree_from_file(boost::property_tree::ptree & pt, const std::string & file);

public:
	//! Property trees of configuration files
	boost::property_tree::ptree common_file_pt, file_pt;

	//! Constructor
	configsrv(const std::string & _dir, const std::string & _ini_file);

	//! Get the config value at the path
	std::string value(const std::string & path) const;

	void change_ini_file(const std::string & _ini_file);
};

#endif /* _CONFIGSRV_H */
