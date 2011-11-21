/*!
 * @brief File containing consts, types and classes related to exceptions specific to ECP_MP
 *
 * @author yoyek

 * @ingroup mp
 */

#ifndef ECP_MP_EXCEPTION_H_
#define ECP_MP_EXCEPTION_H_

#include "base/lib/exception.h"

namespace mrrocpp {
namespace ecp_mp {
namespace exception {

/*!
 * \brief ECP_MP System error
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se, "ECP_MP system_error")

/*!
 * \brief ECP_MP System error
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se_tr, "ECP_MP transmiter system_error")

} // namespace exception
} // namespace ecp_mp
} // namespace mrrocpp

#endif
