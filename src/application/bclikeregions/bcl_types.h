/*
 * bcl_types.h
 *
 *  Created on: Jul 13, 2010
 *      Author: kszkudla
 */

#ifndef BCL_TYPES_H_
#define BCL_TYPES_H_

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

typedef struct {
	std::vector<int> pos;

} regions;

typedef ecp_mp::sensor::fradia_sensor<lib::empty_t, regions> bcl_fradia_sensor;

}

}

}

}

#endif /* BCL_TYPES_H_ */
