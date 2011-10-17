/*!
 * @file exceptions.h
 * @brief File containing consts, types and classes related to exceptions specific for SPKM.
 *
 * @author Tomasz Kornuta
 * @date 08-02-2011
 *
 * @ingroup spkm
 */

#ifndef SPKM_EXCEPTION_H_
#define SPKM_EXCEPTION_H_

#include "base/edp/edp_exceptions.h"
#include "const_spkm.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

//! Desired value that caused the exception.
typedef boost::error_info <struct desired_value_, double> desired_value;

//! Pose specification type.
typedef boost::error_info <struct pose_specification_, mrrocpp::lib::spkm::POSE_SPECIFICATION> pose_specification;

/*!
 * \brief Exception thrown when cartesian pose is required, but unknown.
 * \author Tomasz Kornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_current_cartesian_pose_unknown, "Required current cartesian pose is unknown")

} // namespace spkm
} // namespace edp
} // namespace mrrocpp


#endif /* SPKM_EXCEPTION_H_ */
