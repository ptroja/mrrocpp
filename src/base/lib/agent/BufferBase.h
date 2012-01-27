/*
 * BaseBuffer.h
 *
 *  Created on: Nov 15, 2011
 *      Author: ptroja
 */

#ifndef BUFFERBASE_H_
#define BUFFERBASE_H_

#include <string>

#include <boost/utility.hpp>

namespace mrrocpp {
namespace lib {
namespace agent {

/**
 * Base class for data buffers
 */
class BufferBase : private boost::noncopyable {
	//! name of the data buffer
	const std::string name;

public:
	//! Constructor
	BufferBase(const std::string & _name);

	//! get name of the buffer
	const std::string & getName() const;
};

} // namespace agent
} // namespace lib
} // namespace mrrocpp

#endif /* BUFFERBASE_H_ */
