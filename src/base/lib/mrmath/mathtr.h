/**
 * \file mathtr.h
 *
 * \brief Angle range reduction routines declarations
 *
 * \author Tomasz Kornuta <tkornuta@gmail.com>
 */

#ifndef __MATHTR_H
#define __MATHTR_H

namespace mrrocpp {
namespace lib {

/**
 * Reduce angle value to <-pi,pi> range
 *
 * @bug this actually reduces the value to (-pi,pi>
 * @todo this should be done as default arguments to the 4-arguments variant
 *
 * @param[in] angle value to reduce
 * @return reduced value
 */
double reduce(double angle);

/**
 * Reduce angle value to <min,max) range
 *
 * @todo default arguments: min=-PI, max=PI, offset=2*M_PI
 *
 * @param[in] angle value to reduce
 * @param[in] min lower limit
 * @param[in] max upper limit
 * @param[in] offset reduction offset
 * @return reduced value
 */
double reduce(double angle, double min, double max, double offset);

} // namespace lib
} // namespace mrrocpp

#endif
