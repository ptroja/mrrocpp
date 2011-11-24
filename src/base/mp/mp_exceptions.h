/*!
 * @brief File containing consts, types and classes related to exceptions specific to MP
 *
 * @author yoyek

 * @ingroup mp
 */

#ifndef MP_EXCEPTION_H_
#define MP_EXCEPTION_H_

#include "base/lib/exception.h"

namespace mrrocpp {
namespace mp {
namespace exception {

/*!
 * \brief MP non fatal error
 * \author yoyek
 */
REGISTER_NON_FATAL_ERROR(nfe, "MP non_fatal_error")

/*!
 * \brief MP generator non fatal error
 * \author yoyek
 */
REGISTER_NON_FATAL_ERROR(nfe_g, "MP generator non_fatal_error")

/*!
 * \brief MP robot non fatal error
 * \author yoyek
 */
REGISTER_NON_FATAL_ERROR(nfe_r, "MP generator non_fatal_error")

/*!
 * \brief MP System error
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se, "MP system_error")

} // namespace exception
} // namespace mp
} // namespace mrrocpp

#endif /* EDP_EXCEPTION_H_ */
