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


/**Template class used to make messages from vectors for sending coordinates,
 * and making coordinates vectors from received string messages.
 */
class bcl_message
{
public:
	bcl_message();
	~bcl_message();

	char* trajectoryToString(std::vector<double> vec);

	char* trajectoryToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7);

	std::vector<double> stringToTrajectory(char *str);

	char* fradiaOrderToString(task::fradia_regions& reg, std::vector<double> vec);

	std::vector<double> stringToFradiaOrder(char* str, task::fradia_regions reg);

	void addFradiaOrderToVector(task::fradia_regions& reg, std::vector<task::mrrocpp_regions>& vec);

	char* regionsVectorToString(std::vector<task::mrrocpp_regions> readings, int& num);
};

}

}

}

#endif /* BCL_MESSAGE_H_ */
