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

#define MP_2_ECP_STRING_SIZE 300

bcl_message::bcl_message(){
}

bcl_message::~bcl_message(){
}

//template<class T>
//char* bcl_message::dataToString(std::vector<T> vec){
//	char *ret = new char[MP_2_ECP_STRING_SIZE];
//
//	T* tab = reinterpret_cast<T*>(ret);
//
//	//Write to matrix number of elements which will be written to
//	tab[0] = vec.size();
//
//	int i;
//	//Rewrite vector elements to matrix
//	for(typename std::vector<T>::iterator it = vec.begin(), i = 1; (it != vec.end()) || (i < MP_2_ECP_STRING_SIZE/sizeof(T)); ++it, ++i){
//		tab[i] = *it;
//	}
//
//	return ret;
//}

char* bcl_message::dataToString(std::vector<double> vec){
	char *ret = new char[MP_2_ECP_STRING_SIZE];
//	char *ret = new char[(vec.size() + 1) * sizeof(double)];
//	double *tab = (double*)malloc(sizeof(double) * (vec.size() + 1));//MP_2_ECP_STRING_SIZE);

	double* tab = reinterpret_cast<double*>(ret);

	//Write to matrix number of elements which will be written to
	tab[0] = vec.size();

	int i = 1;
	//Rewrite vector elements to matrix
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

	std::cout << "DATA TO STR " << tab[0]  << std::endl;

//	return reinterpret_cast<char*>(tab);
	return ret;
}

char* bcl_message::dataToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7){
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

std::vector<double> bcl_message::stringToData(char* str){

	std::vector<double> ret;

	double* tab = reinterpret_cast<double*>(str);

//	ret.reserve(size_t(tab[0]));
	ret.clear();

	std::cout << "STRING TO DATA " << ((double *)str)[0] << std::endl;

	ret.assign(tab + 1, tab + (int)tab[0]);


	return ret;
}


//template<class T>
//char* bcl_message::dataToString(T& par0, T& par1, T& par2, T& par3, T& par4, T& par5, T& par6, T& par7){
//	char *ret = new char[MP_2_ECP_STRING_SIZE];
//
//	T* tab = reinterpret_cast<T*>(ret);
//
//	tab[0] = 8;
//
//	tab[1] = par0;
//	tab[2] = par1;
//	tab[3] = par2;
//	tab[4] = par3;
//	tab[5] = par4;
//	tab[6] = par5;
//	tab[7] = par6;
//	tab[8] = par7;
//
//	return ret;
//}
//
//template<class T>
//std::vector<T> bcl_message::stringToData(const char* str, int cnt){
//
//	std::vector<T> ret;
//
//	T* tab = reinterpret_cast<T*>(str);
//
//	ret.reserve(size_t(tab[0]));
//
//	ret.assign(tab + 1, tab + tab[0]);
//
//	return ret;
//}

}

}

}
