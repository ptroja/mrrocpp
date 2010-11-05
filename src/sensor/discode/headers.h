/*
 * headers.h
 *
 *  Created on: Nov 5, 2010
 *      Author: mboryn
 */

#ifndef HEADERS_H_
#define HEADERS_H_

namespace mrrocpp {

namespace ecp_mp {

namespace sensor {

namespace discode {

struct initiate_message_header {
	int data_size;
};

struct reading_message_header {
	int data_size;
};

} // namespace discode

} // namespace sensor

} // namespace ecp_mp

} // namespace mrrocpp

#endif /* HEADERS_H_ */
