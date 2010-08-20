/*
 * bcl_message.h
 *
 *  Created on: Jul 26, 2010
 *      Author: kszkudla
 */

#ifndef BCL_MESSAGE_H_
#define BCL_MESSAGE_H_

#include <vector>
#include "bcl_types.h"

namespace mrrocpp {

namespace ecp {

namespace common {


/**C
 * lass used to make messages from various types of data
 * and data from strings sent as orders
 */
class bcl_message
{
public:
	bcl_message();
	~bcl_message();

	char* robotPositionToString(std::vector<double> vec);

	char* robotPositionToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7);

	std::vector<double> stringToRobotPosition(char *str);

	char* fradiaOrderToString(task::fradia_regions& reg, std::vector<double> vec);

	std::vector<double> stringToFradiaOrder(char* str, task::fradia_regions reg);

	std::vector<double> stringToECPOrder(char* str, std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> > &vec);

};

}

}

}

#endif /* BCL_MESSAGE_H_ */
