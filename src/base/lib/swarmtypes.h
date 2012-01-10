/*
 * swarmtypes.h
 *
 *  Created on: Nov 13, 2011
 *      Author: ptroja
 */

#ifndef SWARMTYPES_H_
#define SWARMTYPES_H_

#include <string>

namespace mrrocpp {
namespace lib {

//! Message sent from ECPs to MP
typedef enum _notification { ACK, NACK } notification_t;

const std::string notifyBufferId = "notification";
const std::string commandBufferId = "command";

}
}

#endif /* SWARMTYPES_H_ */
