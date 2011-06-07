/**
 * \file ecp_mp_message.cc
 * \brief Message serialization class methods definition
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#include <iostream>

#include "ecp_mp_message.h"

namespace mrrocpp {

namespace ecp {

namespace common {


ecp_mp_message::ecp_mp_message(){
}

ecp_mp_message::~ecp_mp_message(){
}


char* ecp_mp_message::robotPositionToString(std::vector<double> vec){
	char *ret = new char[lib::MP_2_ECP_STRING_SIZE];
	double* tab = reinterpret_cast<double*>(ret);

	//Write to matrix number of elements which will be written to
	tab[0] = vec.size();

	int i = 1;
	//Rewrite vector elements to matrix
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < lib::MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

//	std::cout << "DATA TO STR " << tab[0]  << std::endl;

	return ret;
}

char* ecp_mp_message::robotPositionToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7){
	char *ret = new char[lib::MP_2_ECP_STRING_SIZE];

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


std::vector<double> ecp_mp_message::stringToRobotPosition(const uint32_t* str){

	std::cout << "STRING TO DOUBLE TRANSFORM" << std::endl;

	for(int i = 0; i < lib::ECP_2_MP_STRING_SIZE; ++i)
		std::cout << (char)str[i] << ":";
	std::cout << std::endl;

	std::vector<double> ret;

	const double* tab = reinterpret_cast<const double*>(str);

	ret.clear();

	ret.assign(tab + 1, tab + (int)tab[0] + 1);


	return ret;
}


char* ecp_mp_message::fradiaOrderToString(task::fradia_regions& reg, std::vector<double> vec){
	char* ret = new char[lib::MP_2_ECP_STRING_SIZE];

	double *tab = reinterpret_cast<double*>(ret);

	switch(reg.num_found){
		case 4:
			tab[10] = reg.x_k3;
			tab[11] = reg.y_k3;
			tab[12] = reg.r_k3;
		case 3:
			tab[7] = reg.x_k2;
			tab[8] = reg.y_k2;
			tab[9] = reg.r_k2;
		case 2:
			tab[4] = reg.x_k1;
			tab[5] = reg.y_k1;
			tab[6] = reg.r_k1;
		case 1:
			tab[1] = reg.x_k0;
			tab[2] = reg.y_k0;
			tab[3] = reg.r_k0;
		case 0:
			tab[0] = reg.num_found;
			break;
		default:
			tab[0] = 0;
			break;
	}

	tab[VEC_POS - 1] = vec.size();

	int i = VEC_POS;
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < lib::MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

	return ret;
}


std::vector<double> ecp_mp_message::stringToFradiaOrder(char* str, task::fradia_regions reg){
	std::vector<double> ret;
	double *tab = reinterpret_cast<double*>(str);

	switch((int)tab[0]){
		case 4:
			reg.x_k3 = tab[10];
			reg.y_k3 = tab[11];
			reg.r_k3 = tab[12];
		case 3:
			reg.x_k2 = tab[7];
			reg.y_k2 = tab[8];
			reg.r_k2 = tab[9];
		case 2:
			reg.x_k1 = tab[4];
			reg.y_k1 = tab[5];
			reg.r_k1 = tab[6];
		case 1:
			reg.x_k0 = tab[1];
			reg.y_k0 = tab[2];
			reg.r_k0 = tab[3];
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


std::vector<double> ecp_mp_message::stringToECPOrder(const char* str, std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> >& vec){

	task::mrrocpp_regions tmp;

	std::vector<double> ret;

	const double *tab = reinterpret_cast<const double *>(str);

	int size = (int)tab[0] + 1 ;

	ret.assign(tab + 1, tab + 1 + (int)tab[0]);


	for(int i = 0; i < (int)tab[size]; ++i){
		tmp.x = tab[size + 3 * i + 1];
		tmp.y = tab[size + 3 * i + 2];
		tmp.r = tab[size + 3 * i + 3];
		vec.push_back(std::pair<ecp::common::task::mrrocpp_regions, bool>(tmp, false));
		std::cout << "Znaleziono kod: x = " << tmp.x << " y = " << tmp.y << " r = " << tmp.r;
		std::cout << "\a" << std::endl;
	}

	return ret;
}

}

}

}
