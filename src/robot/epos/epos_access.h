/*
 * epos_access_base.h
 *
 *  Created on: Feb 11, 2011
 *      Author: ptroja
 */

#ifndef EPOS_ACCESS_BASE_H_
#define EPOS_ACCESS_BASE_H_

#include <stdint.h>  /* int types with given size */

#include <boost/exception/all.hpp>

namespace mrrocpp {
namespace edp {
namespace epos {

/*
 * Exceptions
 */

//! all high-level methods throws this exception in case of error
struct epos_error : virtual public std::exception, virtual public boost::exception
{
	~epos_error() throw ()
	{
	}
};

//! reason of an exception
typedef boost::error_info <struct tag_reason, std::string> reason;

//! errno code of a failed system call
typedef boost::error_info <struct tag_errno_code, int> errno_code;

//! failed system call
typedef boost::error_info <struct tag_errno_code, std::string> errno_call;

/*!
 * Data types used for communication (Communication Guide reference)
 */

typedef uint32_t DWORD; ///< \brief 32bit type for EPOS data exchange
typedef uint16_t WORD; ///< \brief 16bit type for EPOS data exchange
typedef uint8_t BYTE; ///< \brief 8bit type for EPOS data exchange

class epos_access {
protected:
	//! Flag indicating connection status
	bool device_opened;

public:
	//! EPOS error status
	DWORD E_error;

public:
	//! Constructor
	epos_access();

	//! Destructor
	virtual ~epos_access();

	/*! \brief  send command to EPOS, taking care of all necessary 'ack' and checksum tests
	 *
	 * @param frame array of WORDs to write
	 */
	virtual void sendCommand(WORD *frame) = 0;

	/*! \brief  read an answer frame from EPOS
	 *
	 * @return answer array from the controller
	 */
	virtual unsigned int readAnswer(WORD *ans, unsigned int ans_len) = 0;

	//! Open device
	virtual void open() = 0;

	//! Close device
	virtual void close() = 0;

	/*! \brief Checksum calculation
	 *
	 * Copied from EPOS Communication Guide, p.8
	 *
	 * @param pDataArray pointer to data for checksum calculation
	 * @param numberOfWords length of the data
	 */
	static WORD CalcFieldCRC(const WORD *pDataArray, WORD numberOfWords);
};

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */

#endif /* EPOS_ACCESS_BASE_H_ */
