/*!
 * @file sensor_error.h
 * @brief File containing class representing sensor errors.
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @date Aug 3, 2010
 *
 * @ingroup SENSORS lib
 */

#ifndef SENSOR_ERROR_H_
#define SENSOR_ERROR_H_

#include <exception>

namespace mrrocpp {
namespace lib {
namespace sensor {

/*!
 *
 * @brief Class representing the exceptions thrown by and handled by MRROC++ sensors.
 * @author tkornuta
  *
 * @ingroup SENSORS
 */
class sensor_error : public std::exception
{
public:
	/** Error class. */
	const lib::error_class_t error_class;

	/** Error number. */
	const uint64_t error_no;

	/*!
	 * Constructor.
	 * @param err_cl Error class.
	 * @param err_no Error number.
	 */
	sensor_error(lib::error_class_t err_cl, uint64_t err_no) :
		std::exception(), error_class(err_cl), error_no(err_no)
	{
	}
};

} // namespace sensor
} // namespace lib
} // namespace mrrocpp


#endif /* SENSOR_ERROR_H_ */
