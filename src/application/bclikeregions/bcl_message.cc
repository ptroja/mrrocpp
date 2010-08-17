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


char* bcl_message::trajectoryToString(std::vector<double> vec){
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

char* bcl_message::trajectoryToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7){
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

std::vector<double> bcl_message::stringToTrajectory(char* str){

	std::vector<double> ret;

	double* tab = reinterpret_cast<double*>(str);

	ret.clear();

	std::cout << "STRING TO DATA " << ((double *)str)[0] << std::endl;

	ret.assign(tab + 1, tab + (int)tab[0]);


	return ret;
}

//void bcl_message::addFradiaOrderToVector(task::fradia_regions& reg, std::vector<task::mrrocpp_regions>& vec){
//
//	task::mrrocpp_regions m_reg;
//
//	switch(reg.num_found){
//		case 3:
//			m_reg.x = reg.x_k2;
//			m_reg.y = reg.y_k2;
//			m_reg.w = reg.w_k2;
//			m_reg.h = reg.h_k2;
//			m_reg.a = reg.a_k2;
//			vec.push_back(m_reg);
//		case 2:
//			m_reg.x = reg.x_k1;
//			m_reg.y = reg.y_k1;
//			m_reg.w = reg.w_k1;
//			m_reg.h = reg.h_k1;
//			m_reg.a = reg.a_k1;
//			vec.push_back(m_reg);
//		case 1:
//			m_reg.x = reg.x_k0;
//			m_reg.y = reg.y_k0;
//			m_reg.w = reg.w_k0;
//			m_reg.h = reg.h_k0;
//			m_reg.a = reg.a_k0;
//			vec.push_back(m_reg);
//		default:
//			return;
//	}
//
//
//}

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
 *
 * @param str Order sent from ECP to MP
 * @param vec std::vector to which data is written
 */
void bcl_message::stringToECPOrder(char* str, std::vector<task::mrrocpp_regions>& vec){

	task::mrrocpp_regions tmp;

	double *tab = reinterpret_cast<double *>(str);

	for(int i = 0; i < (int)tab[0]; ++i){
		tmp.x = tab[5 * i + 1];
		tmp.y = tab[5 * i + 2];
		tmp.w = tab[5 * i + 3];
		tmp.h = tab[5 * i + 4];
		tmp.a = tab[5 * i + 5];

		vec.push_back(tmp);
	}
}

//char* bcl_message::regionsVectorToString(std::vector<task::mrrocpp_regions> readings, int& num){
//	char* ret = new char[MP_2_ECP_STRING_SIZE];
//	double *tab = reinterpret_cast<double*>(ret);
//	int cnt = 0;
//
//	std::vector<task::mrrocpp_regions>::iterator it = readings.begin();
//	it += num;
//
//	for(; it != readings.end() && (5*cnt + 1 < MP_2_ECP_STRING_SIZE); ++it){
//		tab[5 * cnt + 1] = (*it).x;
//		tab[5 * cnt + 2] = (*it).y;
//		tab[5 * cnt + 3] = (*it).w;
//		tab[5 * cnt + 4] = (*it).h;
//		tab[5 * cnt + 5] = (*it).a;
//		cnt++;
//	}
//
//	tab[0] = cnt;
//
//	num += 5 * cnt + 1;
//
//	return ret;
//}

}

}

}
