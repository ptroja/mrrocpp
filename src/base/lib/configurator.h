/**
 * \file configurator.h
 *
 * \author Piotr Trojanek <piotr.trojanek@gmail.com>
 * \author Tomasz Winiarski <tomrobotics@gmail.com>
 *
 * \brief Declarations of configurator.
 */

#if !defined(__CONFIGURATOR_H)
#define __CONFIGURATOR_H

#include <iostream>
#include <string>
#include <stdexcept>

#include <sys/utsname.h>
#include <sched.h>

#include <Eigen/Core>

#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/property_tree/exceptions.hpp>

#include "base/lib/messip/messip_dataport.h"
#include "base/lib/config_types.h"
#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

class configurator
{
private:
	//! Node name of the installation folder
	const std::string node;

	//! Installation directory
	const std::string dir;

	//! Installation network path
	std::string mrrocpp_network_path;

	//! System info
	struct utsname sysinfo;

	//! Mutex to protect exclusive access
	mutable boost::mutex access_mutex;

	//! Communication channel to the configuration server
	messip_channel_t *ch;

public:

	//! returns sr attach point
	std::string get_sr_attach_point() const;

	//! returns ui attach point
	std::string get_ui_attach_point() const;

	//! returns mp_pulse_attach_point point
	std::string get_mp_pulse_attach_point() const;

	//! returns edp_section of the robot
	std::string get_edp_section(const robot_name_t & _robot_name) const;

	//! returns edp_section of the robot
	std::string get_edp_section() const;

	//! returns ecp_section of the robot
	std::string get_ecp_section(const robot_name_t & _robot_name) const;

	//! returns ecp_section of the robot
	std::string get_ecp_section() const;

	//! returns ecp_section of the robot
	std::string get_ecp_trigger_attach_point(const robot_name_t & _robot_name) const;

	//! returns ecp_section of the robot
	std::string get_ecp_trigger_attach_point() const;

	//! returns ecp_section of the robot
	std::string get_ecp_attach_point(const robot_name_t & _robot_name) const;

	//! returns ecp_section of the robot
	std::string get_ecp_attach_point() const;

	//! returns ecp_section of the robot
	std::string get_edp_hardware_busy_file(const robot_name_t & _robot_name) const;

	//! returns ecp_section of the robot
	std::string get_edp_hardware_busy_file() const;

	//! returns ecp_section of the robot
	std::string get_edp_reader_attach_point(const robot_name_t & _robot_name) const;

	//! returns ecp_section of the robot
	std::string get_edp_reader_attach_point() const;

	//! returns ecp_section of the robot
	std::string get_edp_resourceman_attach_point(const robot_name_t & _robot_name) const;

	//! returns ecp_section of the robot
	std::string get_edp_resourceman_attach_point() const;

	/**
	 * Get network path to the installed MRROC++ system
	 * @return folder path
	 */
	std::string return_mrrocpp_network_path() const;

	/**
	 * Get path to the default reader measures folder
	 * @return folder path
	 */
	std::string return_default_reader_measures_path() const;

	/**
	 * Configuration section name
	 */
	const std::string section_name;

	/**
	 * Configuration robot_name
	 * it can be extracted from the section names for ecp and edp sections
	 */
	std::string robot_name;

	/**
	 * Check if key non-zero key exist in the configuration file
	 * @bug does not protect from changing configuration during check
	 * @param key
	 * @return
	 */
	bool check_config(const std::string & key) const;

	/**
	 * Constructor
	 * @param _node node of the install folder
	 * @param _dir directory to the install folder
	 * @param _section_name configuration section name
	 */
	configurator(const std::string & _node, const std::string & _dir, const std::string & _section_name);

	/**
	 * Change configuration file
	 * @param _ini_file new configuration file
	 */
	void change_config_file(const std::string & _ini_file);

	/**
	 * Spawn new process
	 * @param _section_name configuration section of the process to spawn
	 * @return
	 */
	pid_t process_spawn(const std::string & _section_name);

	/**
	 * Return network attach point
	 * @param _type type of the network path
	 * @param _key configuration key
	 * @param __section_name section name
	 * @return network path
	 */
	std::string	return_attach_point_name(const char* _key, const char* __section_name = NULL) const;

	/**
	 * Return network attach point
	 * @param _type type of the network path
	 * @param _key configuration key
	 * @param __section_name section name
	 * @return network path
	 */
	std::string return_attach_point_name(const std::string & _key, const std::string & __section_name) const
	{
		return return_attach_point_name(_key.c_str(), __section_name.c_str());
	}

	/**
	 * Get value from the configuration
	 * @param Type typename of the requested value
	 * @param _key configuration key
	 * @param __section_name section name
	 * @return configuration value
	 */
	template <class Type>
	Type value(const std::string & _key, const std::string & __section_name) const
	{
		// initialize property tree path
		std::string pt_path = __section_name;

		// trim leading '[' char
		pt_path.erase(0, 1);
		// trim trailing '[' char
		pt_path.erase(pt_path.length() - 1, 1);

		pt_path += ".";
		pt_path += _key;

		boost::mutex::scoped_lock l(access_mutex);

		config_query_t query, reply;

		query.key = pt_path;
		query.flag = false;

		messip::port_send(this->ch, 0, 0, query, reply);

		if (reply.flag) {
			return boost::lexical_cast <Type>(reply.key);
		} else {
			throw boost::property_tree::ptree_error("remote config query failed: probably missing key \""
					+ __section_name + "." + _key + "\" in config file. pt_path=\"" + pt_path + "\".");
		}

	}

	/**
	 * Get value from the configuration with the default section name
	 * @param Type typename of the requested value
	 * @param _key configuration key
	 * @return configuration value
	 */
	template <class Type>
	Type value(const std::string & _key) const
	{
		return value <Type> (_key, section_name);
	}

	/**
	 * Read matrix from config.
	 * Matrix has format similar to MatLAB, for example: [ a b c; d e f; g h i].
	 * @param key
	 * @param section_name
	 * @return
	 */
	template <int ROWS, int COLS>
	Eigen::Matrix <double, ROWS, COLS> value(const std::string & key, const std::string & section_name) const;

	/**
	 * Check is non-zero configuration value exist
	 * @param _key key
	 * @param __section_name section name
	 * @return true if the non-zero value exists
	 */
	bool exists(const std::string & _key) const;

	/**
	 * Check is non-zero configuration value exist
	 * @param _key key
	 * @param __section_name section name
	 * @return true if the non-zero value exists
	 */
	bool exists(const std::string & _key, const std::string & __section_name) const;

	/**
	 * Check is non-zero configuration value exist and is true
	 * @param _key key
	 * @param __section_name section name
	 * @return true if the non-zero value exists
	 */
	bool exists_and_true(const char* _key, const char* __section_name = NULL) const;

	/**
	 * Check is non-zero configuration value exist and is true
	 * @param _key key
	 * @param __section_name section name
	 * @return true if the non-zero value exists
	 */
	bool exists_and_true(const std::string & _key, const std::string & __section_name) const
	{
		return exists_and_true(_key.c_str(), __section_name.c_str());
	}

	/**
	 * Check is non-zero configuration value exist and is true
	 * @param _key key
	 * @return true if the non-zero value exists
	 */
	bool exists_and_true(const std::string & _key) const
	{
		return exists_and_true(_key, section_name);
	}

	//! Destructor
	~configurator();

protected:
	/**
	 * Extract elements from matrix row. For example: string " 1   2 3   4 " will be interpreted as a vector {1,2,3,4}
	 */
	template <int COLS>
	Eigen::Matrix <double, 1, COLS> get_vector_elements(std::string text_value) const;
};

template <int COLS>
Eigen::Matrix <double, 1, COLS> configurator::get_vector_elements(std::string text_value) const
{
	//	std::cout << "configurator::get_vector_elements() begin\n";
	Eigen::Matrix <double, 1, COLS> row_value;
	const char * blank_chars = { " \t\n\r" };
	boost::algorithm::trim_if(text_value, boost::algorithm::is_any_of(blank_chars));

	// split it into elements
	boost::char_separator <char> space_separator(blank_chars);
	boost::tokenizer <boost::char_separator <char> > tok(text_value, space_separator);

	int element_no = 0;
	for (boost::tokenizer <boost::char_separator <char> >::iterator it = tok.begin(); it != tok.end(); ++it, ++element_no) {
		//std::cout << " " << *it << ", ";
		if (element_no >= COLS) {
			throw std::logic_error("configurator::get_vector_elements(): vector has more elements than expected: \""
					+ text_value + "\".");
		}
		std::string element = *it;
		boost::algorithm::trim(element);
		double element_value = boost::lexical_cast <double>(element);
		row_value(0, element_no) = element_value;
	}

	if (element_no != COLS) {
		throw std::logic_error("configurator::get_vector_elements(): vector has less elements than expected: \""
				+ text_value + "\".");
	}
	//	std::cout << "configurator::get_vector_elements() end\n";
	return row_value;
}

template <int ROWS, int COLS>
Eigen::Matrix <double, ROWS, COLS> configurator::value(const std::string & key, const std::string & section_name) const
{
	//	std::cout << "configurator::get_matrix_value() begin\n";
	Eigen::Matrix <double, ROWS, COLS> matrix_value;

	// get string value and remove leading and trailing spaces
	std::string text_value = value <std::string> (key, section_name);
	boost::algorithm::trim(text_value);

	//std::cout << "visual_servo_regulator::get_matrix_value() Processing value: "<<text_value<<"\n";

	// check for [ and ], and then remove it
	if (text_value.size() < 3 || text_value[0] != '[' || text_value[text_value.size() - 1] != ']') {
		throw std::logic_error("configurator::value(): leading or trailing chars [] not found or no value supplied for parameter: \""
				+ key + "\" in section \"" + section_name + "\"");
	}
	boost::algorithm::trim_if(text_value, boost::algorithm::is_any_of("[]"));

	boost::char_separator <char> semicolon_separator(";");
	boost::tokenizer <boost::char_separator <char> > tok(text_value, semicolon_separator);

	int row_no = 0;
	for (boost::tokenizer <boost::char_separator <char> >::iterator it = tok.begin(); it != tok.end(); ++it, ++row_no) {
		if (row_no >= ROWS) {
			throw std::logic_error("configurator::value(): matrix has more rows than expected. Parameter: \"" + key
					+ "\" in section \"" + section_name + "\"");
		}
		matrix_value.row(row_no) = get_vector_elements <COLS> (*it);
	}
	if (row_no != ROWS) {
		throw std::logic_error("configurator::value(): matrix has more rows than expected. Parameter: \"" + key
				+ "\" in section \"" + section_name + "\"");
	}

	return matrix_value;
}

} // namespace lib
} // namespace mrrocpp

#endif
