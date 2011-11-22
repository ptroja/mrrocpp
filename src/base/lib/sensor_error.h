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

#include "exception.h"

namespace mrrocpp {
namespace lib {
namespace exception {

/*!
 * \brief Sensor System error
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se_sensor, "Sensor System_error")

/*!
 * \brief Sensor System error
 * \author yoyek
 */
REGISTER_FATAL_ERROR(fe_sensor, "Sensor Fatal_error")

} // namespace sensor
} // namespace lib
} // namespace mrrocpp

#endif /* SENSOR_ERROR_H_ */
