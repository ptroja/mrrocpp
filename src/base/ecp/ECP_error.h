/*
 * ECP_error.h
 *
 *  Created on: Feb 25, 2011
 *      Author: ptroja
 */

#ifndef ECP_ERROR_H_
#define ECP_ERROR_H_

#include "base/lib/com_buf.h"
#include "base/lib/exception.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief ECP generator error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ECP_error
{
public:
	/**
	 * @brief error class (type)
	 */
	const lib::error_class_t error_class;

	/**
	 * @brief error number
	 */
	const uint64_t error_no;

	/**
	 * @brief edp error structure
	 */
	lib::edp_error error;

	/**
	 * @brief constructor
	 * @param err_cl error class
	 * @param err_no error number
	 * @param err0 EDP error0 number
	 * @param err1 EDP error1 number
	 */
	ECP_error(lib::error_class_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_ERROR_H_ */
