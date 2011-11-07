/*!
 * @file epos_exceptions.hpp
 * @brief Boost::exception based exceptions thrown in case of errors/problems during control of the EPOS.
 *
 * @author tkornuta
 * @date 30-05-2011
 *
 * @ingroup LIB
 */

#ifndef EPOS_EXCEPTIONS_HPP_
#define EPOS_EXCEPTIONS_HPP_

#include "base/lib/exception.h"

namespace mrrocpp {
namespace edp {
namespace epos {

/*!
 * \brief Exception thrown in case of error of the interpolation buffer.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_epos_interpolation_buffer, "Interpolation Buffer Error")


} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */


#endif /* EPOS_EXCEPTIONS_HPP_ */
