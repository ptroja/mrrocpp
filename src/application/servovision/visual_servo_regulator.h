/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_REGULATOR_H_
#define VISUAL_SERVO_REGULATOR_H_

#include <string>

#include "lib/configurator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/**
 * Abstract class for regulators used in servovision. Regulator calculates control supplied to servovision generator
 */
class visual_servo_regulator
{
public:
	virtual ~visual_servo_regulator()
	{
	}
	;

	virtual const boost::numeric::ublas::vector<double> & calculate_control(const boost::numeric::ublas::vector<double> & error) = 0;

	const boost::numeric::ublas::vector<double> & get_control() { return calculated_control; }

protected:
	visual_servo_regulator(const lib::configurator & config, const char * config_section_name, int error_size, int control_size);

	const lib::configurator & config;

	const std::string config_section_name;

	/**
	 * Extract elements from vector. For example: " 1   2 3   4 "
	 */
	//boost::numeric::ublas::vector<double> get_vector_elements(std::string text_value, int n);

	/**
	 * Read vector from config. Vector has format similar to MatLAB, for example: [ x y z ].
	 * @param name
	 * @param n vector size
	 * @return vector read
	 * @throws exception if vector has not been read
	 */
	//boost::numeric::ublas::vector<double> get_vector_value(const std::string & key, int n);

	/**
	 * Read matrix from config. Matrix has format similar to MatLAB, for example: [ a b c d; e f g h ].
	 * @param name
	 * @param n matrix size - rows
	 * @param m matrix size - columns
	 * @return vector read
	 * @throws exception if vector has not been read
	 */
	//boost::numeric::ublas::matrix<double> get_matrix_value(const std::string & key, int n, int m);

	boost::numeric::ublas::vector<double> calculated_control;

	const int error_size;
	const int control_size;
private:

}; // class visual_servo_regulator

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* VISUAL_SERVO_REGULATOR_H_ */
