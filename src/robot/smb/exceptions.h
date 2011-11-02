/*!
 * @file exceptions.h
 * @brief File containing consts, types and classes related to exceptions specific for SMB.
 *
 * @author Tomasz Kornuta
 * @date 10-10-2011
 *
 * @ingroup smb
 */


#ifndef SMB_EXCEPTIONS_H_
#define SMB_EXCEPTIONS_H_

#include "base/edp/edp_exceptions.h"
#include "const_smb.h"

namespace mrrocpp {
namespace edp {
namespace smb {

//! Desired value that caused the exception.
typedef boost::error_info <struct desired_value_, double> desired_value;

//! Pose specification type.
typedef boost::error_info <struct pose_specification_, mrrocpp::lib::smb::POSE_SPECIFICATION> pose_specification;


//! Pose specification type.
typedef boost::error_info <struct current_state_, mrrocpp::lib::smb::ALL_LEGS_VARIANT> current_state;

//! Pose specification type.
typedef boost::error_info <struct festo_command_, mrrocpp::lib::smb::ALL_LEGS_VARIANT> retrieved_festo_command;

/*!
 * \brief Exception thrown in case of invalid command received in given SMB state.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_command_in_given_state, "Invalid command in given SMB state")

/*!
 * \brief Exception thrown in case of receiving of command requiring the clamps rotation when it is prohibited.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_clamps_rotation_prohibited_in_given_state, "Clamps rotation prohibited in given state")



} // namespace spkm
} // namespace edp
} // namespace mrrocpp




#endif /* SMB_EXCEPTIONS_H_ */
