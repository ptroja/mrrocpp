/*
 * bcl_message.h
 *
 *  Created on: Jul 26, 2010
 *      Author: kszkudla
 */

#ifndef BCL_MESSAGE_H_
#define BCL_MESSAGE_H_

#include <vector>

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

//	template <class T>
//	char* dataToString(std::vector<T> vec);
//
//	template <class T>
//	char* dataToString(T& par0, T& par1, T& par2, T& par3, T& par4, T& par5, T& par6, T& par7);
//
//	template <class T>
//	std::vector<T> stringToData(const char *str, int cnt);

	char* dataToString(std::vector<double> vec);

	char* dataToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7);

	std::vector<double> stringToData(char *str);
};

}

}

}

#endif /* BCL_MESSAGE_H_ */
