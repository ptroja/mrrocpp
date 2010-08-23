/*
 * bcl_message.cc
 *
 *  Created on: Jul 26, 2010
 *      Author: kszkudla
 */

#include "bcl_message.h"
#include <iostream>

namespace mrrocpp {

namespace ecp {

namespace common {


bcl_message::bcl_message(){
}

bcl_message::~bcl_message(){
}

/**
 * Function converting robot position held in vector to char matrix
 * @param vec vector with data to send
 * @return matrix of chars, size MP_2_ECP_STRING_SIZE, containing data from vector
 */
char* bcl_message::robotPositionToString(std::vector<double> vec){
	char *ret = new char[MP_2_ECP_STRING_SIZE];
	double* tab = reinterpret_cast<double*>(ret);

	//Write to matrix number of elements which will be written to
	tab[0] = vec.size();

	int i = 1;
	//Rewrite vector elements to matrix
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

//	std::cout << "DATA TO STR " << tab[0]  << std::endl;

	return ret;
}

/**
 * Function converting robot position given as seven parameters to char matrix
 * @param vec vector with data to send
 * @return matrix of chars, size MP_2_ECP_STRING_SIZE, containing data from vector
 */
char* bcl_message::robotPositionToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7){
	char *ret = new char[MP_2_ECP_STRING_SIZE];

	double* tab = reinterpret_cast<double*>(ret);

	tab[0] = 8;

	tab[1] = par0;
	tab[2] = par1;
	tab[3] = par2;
	tab[4] = par3;
	tab[5] = par4;
	tab[6] = par5;
	tab[7] = par6;
	tab[8] = par7;

	return ret;
}

/**
 * Method converting matrix of char to std::vector with robot position
 * @param str pointer to matrix first element
 * @return vector with read position
 */
std::vector<double> bcl_message::stringToRobotPosition(char* str){

	std::vector<double> ret;

	double* tab = reinterpret_cast<double*>(str);

	ret.clear();

	ret.assign(tab + 1, tab + (int)tab[0] + 1);


	return ret;
}

/**
 * Method converting MRROC++<->FraDIA communication structure and
 * actual robot position to matrix of char
 * @param reg communication structure to parse
 * @param vec vectorcontaining robot's position
 * @return matrix of chars, size MP_2_ECP_STRING_SIZE, containing data from vector and structure
 */
char* bcl_message::fradiaOrderToString(task::fradia_regions& reg, std::vector<double> vec){
	char* ret = new char[MP_2_ECP_STRING_SIZE];

	double *tab = reinterpret_cast<double*>(ret);

	switch(reg.num_found){
//		case 5:
//			tab[21] = reg.x_k4;
//			tab[22] = reg.y_k4;
//			tab[23] = reg.w_k4;
//			tab[24] = reg.h_k4;
//			tab[25] = reg.a_k3;
//		case 4:
//			tab[16] = reg.x_k3;
//			tab[17] = reg.y_k3;
//			tab[18] = reg.w_k3;
//			tab[19] = reg.h_k3;
//			tab[20] = reg.a_k3;
		case 3:
			tab[11] = reg.x_k2;
			tab[12] = reg.y_k2;
			tab[13] = reg.w_k2;
			tab[14] = reg.h_k2;
			tab[15] = reg.a_k2;
		case 2:
			tab[6] = reg.x_k1;
			tab[7] = reg.y_k1;
			tab[8] = reg.w_k1;
			tab[9] = reg.h_k1;
			tab[10] = reg.a_k1;
		case 1:
			tab[1] = reg.x_k0;
			tab[2] = reg.y_k0;
			tab[3] = reg.w_k0;
			tab[4] = reg.h_k0;
			tab[5] = reg.a_k0;
		case 0:
			tab[0] = reg.num_found;
			break;
		default:
			tab[0] = 0;
			break;
	}

	tab[VEC_POS - 1] = vec.size();

	int i = VEC_POS;
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

	return ret;
}

/**
 * Method to convert data from matrix of char to MRROC++<->FraDIA communication strucure
 * and vector containing robot's position
 * @param str pointer to matrix first element
 * @param reg structure to which data will be written
 * @return vector containing robot's position
 */
std::vector<double> bcl_message::stringToFradiaOrder(char* str, task::fradia_regions reg){
	std::vector<double> ret;
	double *tab = reinterpret_cast<double*>(str);

	switch((int)tab[0]){
//		case 5:
//			reg.x_k4 = tab[21];
//			reg.y_k4 = tab[22];
//			reg.w_k4 = tab[23];
//			reg.h_k4 = tab[24];
//			reg.a_k4 = tab[25];
//		case 4:
//			reg.x_k3 = tab[16];
//			reg.y_k3 = tab[17];
//			reg.w_k3 = tab[18];
//			reg.h_k3 = tab[19];
//			reg.a_k3 = tab[20];
		case 3:
			reg.x_k2 = tab[11];
			reg.y_k2 = tab[12];
			reg.w_k2 = tab[13];
			reg.h_k2 = tab[14];
			reg.a_k2 = tab[15];
		case 2:
			reg.x_k1 = tab[6];
			reg.y_k1 = tab[7];
			reg.w_k1 = tab[8];
			reg.h_k1 = tab[9];
			reg.a_k1 = tab[10];
		case 1:
			reg.x_k0 = tab[1];
			reg.y_k0 = tab[2];
			reg.w_k0 = tab[3];
			reg.h_k0 = tab[4];
			reg.a_k0 = tab[5];
			reg.code_found = true;
			reg.num_found = tab[0];
			ret.assign(tab + VEC_POS, tab + (int)tab[VEC_POS - 1]);
			break;
		case 0:
		default:
			ret.clear();
			reg.code_found = false;
			break;

	}

	return ret;
}

/**
 * Method to convert matrix of char to vecotrs containing robot's position and
 * founded regions data.
 * @param str Order sent from ECP to MP
 * @param vec std::vector to which regions data will be written
 * @return vecotr containing robot's position
 */
std::vector<double> bcl_message::stringToECPOrder(char* str, std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> >& vec){

	task::mrrocpp_regions tmp;

	std::vector<double> ret;

	double *tab = reinterpret_cast<double *>(str);

	int size = (int)tab[0] + 1;

	ret.assign(tab + 1, tab + size);


	for(int i = size; i < (int)tab[size]; ++i){
		tmp.x = tab[size + 5 * i + 1];
		tmp.y = tab[size + 5 * i + 2];
		tmp.w = tab[size + 5 * i + 3];
		tmp.h = tab[size + 5 * i + 4];
		tmp.a = tab[size + 5 * i + 5];

		vec.push_back(std::pair<ecp::common::task::mrrocpp_regions, bool>(tmp, false));
	}

	return ret;
}

}

}

}
