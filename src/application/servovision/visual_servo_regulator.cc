/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

//#include <boost/algorithm/string/trim.hpp>
//#include <boost/algorithm/string/classification.hpp>
//#include <stdexcept>
//#include <iostream>

#include "visual_servo_regulator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

//typedef boost::tokenizer <boost::char_separator <char> > tokenizer;

visual_servo_regulator::visual_servo_regulator(const lib::configurator & config, const char * config_section_name, int error_size, int control_size) :
	config(config), config_section_name(config_section_name), calculated_control(control_size), error_size(error_size),
			control_size(control_size)
{

}

/*boost::numeric::ublas::vector <double> visual_servo_regulator::get_vector_elements(std::string text_value, int n)
{
	//std::cout << "visual_servo_regulator::get_vector_elements() begin\n";
	boost::numeric::ublas::vector <double> value(n);
	const char * blank_chars = { " \t\n\r" };
	boost::algorithm::trim_if(text_value, boost::algorithm::is_any_of(blank_chars));

	// split it into elements
	boost::char_separator <char> space_separator(blank_chars);
	tokenizer tok(text_value, space_separator);

	int element_no = 0;
	for (tokenizer::iterator it = tok.begin(); it != tok.end(); ++it, ++element_no) {
		//std::cout << " " << *it << ", ";
		if (element_no > n) {
			throw std::logic_error("visual_servo_regulator::get_vector_elements(): element_no > n");
		}
		std::string element = *it;
		boost::algorithm::trim(element);
		double element_value = boost::lexical_cast <double>(element);
		value(element_no) = element_value;
	}

	if (element_no != n) {
		throw std::logic_error("visual_servo_regulator::get_vector_elements(): element_no != n");
	}
	return value;
}

boost::numeric::ublas::vector <double> visual_servo_regulator::get_vector_value(const std::string & key, int n)
{
	//std::cout << "visual_servo_regulator::get_vector_value() begin\n";
	// get string value and remove leading and trailing spaces
	std::string text_value = config.value <std::string> (key, config_section_name);
	boost::algorithm::trim(text_value);

	// check for [ and ], and then remove it
	if (text_value.size() < 3 || text_value[0] != '[' || text_value[text_value.size() - 1] != ']') {
		throw std::logic_error("visual_servo_regulator::get_vector_value(): []");
	}

	boost::algorithm::trim_if(text_value, boost::algorithm::is_any_of("[]"));
	return get_vector_elements(text_value, n);
}

boost::numeric::ublas::matrix <double> visual_servo_regulator::get_matrix_value(const std::string & key, int n, int m)
{
	//std::cout << "visual_servo_regulator::get_matrix_value() begin\n";
	boost::numeric::ublas::matrix <double> value(n, m);

	// get string value and remove leading and trailing spaces
	std::string text_value = config.value <std::string> (key, config_section_name);
	boost::algorithm::trim(text_value);

	//std::cout << "visual_servo_regulator::get_matrix_value() Processing value: "<<text_value<<"\n";

	// check for [ and ], and then remove it
	if (text_value.size() < 3 || text_value[0] != '[' || text_value[text_value.size() - 1] != ']') {
		throw std::logic_error("visual_servo_regulator::get_matrix_value(): text_value.size() < 3");
	}
	boost::algorithm::trim_if(text_value, boost::algorithm::is_any_of("[]"));

	boost::char_separator <char> semicolon_separator(";");
	tokenizer tok(text_value, semicolon_separator);

	int row_no = 0;
	for (tokenizer::iterator it = tok.begin(); it != tok.end(); ++it, ++row_no) {
		if (row_no > n) {
			throw std::logic_error("visual_servo_regulator::get_matrix_value(): row_no > n");
		}
		boost::numeric::ublas::vector <double> row = get_vector_elements(*it, m);
		for (int i = 0; i < m; ++i) {
			value(row_no, i) = row(i);
		}
	}
	if (row_no != n) {
		throw std::logic_error("visual_servo_regulator::get_matrix_value(): row_no != n");
	}
	return value;
}*/

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

