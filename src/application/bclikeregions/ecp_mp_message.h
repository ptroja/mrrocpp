/**
 * \file ecp_mp_message.h
 * \brief Message serialization class
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#ifndef ECP_MP_MESSAGE_H_
#define ECP_MP_MESSAGE_H_

#include <vector>
#include "bcl_types.h"

namespace mrrocpp {

namespace ecp {

namespace common {


/**C
 * lass used to make messages from various types of data
 * and data from strings sent as orders
 */
class ecp_mp_message
{
public:
	/**
	 * Class contructor
	 */
	ecp_mp_message();
	/**
	 * Class destructor
	 */
	~ecp_mp_message();

	/**
	 * Function converting robot position held in vector to char matrix
	 * @param vec vector with data to send
	 * @return matrix of chars, size MP_2_ECP_STRING_SIZE, containing data from vector
	 */
	char* robotPositionToString(std::vector<double> vec);
	/**
	 * Function converting robot position given as seven parameters to char matrix
	 * @param vec vector with data to send
	 * @return matrix of chars, size MP_2_ECP_STRING_SIZE, containing data from vector
	 */
	char* robotPositionToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7);
	/**
	 * Method converting matrix of char to std::vector with robot position
	 * @param str pointer to matrix first element
	 * @return vector with read position
	 */
	std::vector<double> stringToRobotPosition(const uint32_t* str);
	/**
	 * Method converting MRROC++<->FraDIA communication structure and
	 * actual robot position to matrix of char
	 * @param reg communication structure to parse
	 * @param vec vectorcontaining robot's position
	 * @return matrix of chars, size MP_2_ECP_STRING_SIZE, containing data from vector and structure
	 */
	char* fradiaOrderToString(task::fradia_regions& reg, std::vector<double> vec);
	/**
	 * Method to convert data from matrix of char to MRROC++<->FraDIA communication strucure
	 * and vector containing robot's position
	 * @param str pointer to matrix first element
	 * @param reg structure to which data will be written
	 * @return vector containing robot's position
	 */
	std::vector<double> stringToFradiaOrder(char* str, task::fradia_regions reg);
	/**
	 * Method to convert matrix of char to vecotrs containing robot's position and
	 * founded regions data.
	 * @param str Order sent from ECP to MP
	 * @param vec std::vector to which regions data will be written
	 * @return vecotr containing robot's position
	 */
	std::vector<double> stringToECPOrder(const char* str, std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> > &vec);

};

}

}

}

#endif /* ECP_MP_MESSAGE_H_ */
