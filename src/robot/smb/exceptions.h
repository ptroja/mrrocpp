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

//! Convert exception's to human-readable string
inline std::string to_string(pose_specification const & e)
{
	switch(e.value()) {
		case lib::smb::EXTERNAL:
			return std::string("EXTERNAL");
		case lib::smb::JOINT:
			return std::string("JOINT");
		case lib::smb::MOTOR:
			return std::string("MOTOR");
		default:
			break;
	}

	return std::string("UNKNOWN");
}

inline std::string to_string(lib::smb::ALL_LEGS_VARIANT v)
{
	switch(v) {
		case lib::smb::ALL_OUT:
			return std::string("ALL_OUT");
		case lib::smb::ALL_IN:
			return std::string("ALL_IN");
		case lib::smb::ONE_IN_TWO_OUT:
			return std::string("ONE_IN_TWO_OUT");
		case lib::smb::TWO_IN_ONE_OUT:
			return std::string("TWO_IN_ONE_OUT");
		default:
			break;
	}

	return std::string("UNKNOWN");
}

//! Convert exception's to human-readable string
inline std::string to_string(current_state const & e)
{
	return to_string(e.value());
}

//! Convert exception's to human-readable string
inline std::string to_string(retrieved_festo_command const & e)
{
	return to_string(e.value());
}

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

/*!
 * \brief Unexpected case within a 'switch' statement with a 'default' handler.
 * \author ptroja
 */
REGISTER_FATAL_ERROR(unexpected_case_within_switch, "Unexpected case within a 'switch' statement with a 'default' handler")

} // namespace spkm
} // namespace edp
} // namespace mrrocpp

#endif /* SMB_EXCEPTIONS_H_ */
