/*!
 * @file pvat_exceptions.hpp
 * @brief Errors based on boost::exception thrown during the Cartesian trajectory generation.
 *
 *
 * @author tkornuta
 * @date 16-05-2011
 *
 * @ingroup LIB
 */

#include "base/edp/edp_exceptions.h"

#ifndef PVAT_EXCEPTIONS_HPP_
#define PVAT_EXCEPTIONS_HPP_

//! Name of violated constraint - maximum.
const std::string MAXIMUM_CONSTRAINT = "Maximum";

//! Name of violated constraint - minimum.
const std::string MINIMUM_CONSTRAINT = "Minimum";

//! Type of violated constraint.
typedef boost::error_info <struct constraint_type_, std::string> constraint_type;

//! Number of motor that caused the exception.
typedef boost::error_info <struct motor_number_, int> motor_number;

//! Segment in which a constraint was exceeded.
typedef boost::error_info <struct segment_number_, int> segment_number;

//! Desired value that caused the exception.
typedef boost::error_info <struct desired_value_, double> desired_value;

//! Constraint value that caused the exception.
typedef boost::error_info <struct desired_value_, double> constraint_value;

/*!
 * \brief Exception thrown when motor velocity constraint exceeded.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_motor_velocity_constraint_exceeded, "Motor velocity constraint exceeded")

/*!
 * \brief Exception thrown when motor velocity constraint exceeded.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_motor_acceleration_constraint_exceeded, "Motor acceleration constraint exceeded")

#endif /* PVAT_EXCEPTIONS_HPP_ */
