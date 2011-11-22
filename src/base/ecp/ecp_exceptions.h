/*!
 * @brief File containing consts, types and classes related to exceptions specific to ECP
 *
 * @author yoyek

 * @ingroup ecp
 */

#ifndef ECP_EXCEPTION_H_
#define ECP_EXCEPTION_H_

#include "base/lib/exception.h"

namespace mrrocpp {
namespace ecp {
namespace exception {

/*!
 * \brief ECP robot non fatal error
 * \author yoyek
 */
REGISTER_NON_FATAL_ERROR(nfe_r, "ECP robot non_fatal_error")

/*!
 * \brief ECP robot fatal error
 * \author yoyek
 */
REGISTER_FATAL_ERROR(fe_r, "ECP robot fatal_error")

/*!
 * \brief ECP robot System error
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se_r, "ECP robot system_error")

/*!
 * \brief ECP generator non fatal error
 * \author yoyek
 */
REGISTER_NON_FATAL_ERROR(nfe_g, "ECP generator non_fatal_error")

/*!
 * \brief ECP generator System error
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se_g, "ECP generator system_error")

/*!
 * \brief ECP System error
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se, "ECP system_error")

} // namespace exception
} // namespace ecp
} // namespace mrrocpp

#endif
