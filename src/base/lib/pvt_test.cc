/*!
 * @file pvt_test.cc
 * @brief Position-Velocity-Time triples calculation for Maxon EPOS motino controller - test utility.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <stdio.h>
#include <iostream>

#include <Eigen/Core>

#include "pvt.hpp"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

int
main(int argc, char *argv[])
{
	Matrix<double,1,1> Delta, Vmax, Amax, Vnew, Anew, Dnew;

	Delta(0,0) = 100; // position delta
	Vmax(0,0) = 2; // maximal velotivy
	Amax(0,0) = 1; // maximal acceleration

	//const double t1 = pvt<1>(Goal, Vmax, Amax);
	const double t2 = ppm<1>(Delta, Vmax, Amax, Vnew, Anew, Dnew);

	std::cerr <<
		"Vnew:\n" << Vnew << std::endl <<
		"Anew:\n" << Anew << std::endl <<
		"Dnew:\n" << Dnew << std::endl <<
		std::endl;

	std::cerr << "ppm: " << t2 << std::endl;

	//printf("pvt: %f\tppm: %f\n", t1, t2);

	return 0;
}
