/*!
 * @file pvt_test.cc
 * @brief Position-Velocity-Time triples calculation for Maxon EPOS motino controller - test utility.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <iostream>
#include <stdint.h>

#include <Eigen/Core>

#include "pvt.hpp"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

const uint32_t _Vdefault[6] = { 5000UL, 5000UL, 5000UL, 5000UL, 5000UL, 5000UL };
const uint32_t _Adefault[6] = { 30000UL, 30000UL, 30000UL, 30000UL, 15000UL, 30000UL };
const uint32_t _Ddefault[6] = { 30000UL, 30000UL, 30000UL, 30000UL, 15000UL, 30000UL };

//! Initialization of the encoder resolution. Equals to the Counts Per Turn (CPT) x 4.
const uint32_t encoder_resolution[6] = {
		500*4, 500*4, 500*4,
		2000*4, 2000*4, 2000*4
};

int
main(int argc, char *argv[])
{
	Eigen::Matrix<double,6,1> Delta, Vmax, Amax, Vnew, Anew, Dnew;
	Delta << +90.36, +87.8, +112.6, +0.2425, +11.77, +2.462;
	Vmax << +83.33, +83.33 ,+83.33, +83.33, +50, +83.33;
	Amax << +833.3, +833.3, +833.3, +166.7, +100, +500;

	const double t2 = ppm<6>(Delta, Vmax, Amax, Vnew, Anew, Dnew);

	std::cerr.precision(5);
	std::cerr
			<< "Delta:\n" << Delta.transpose() << std::endl
			<< "Vmax:\n" << Vmax.transpose() << std::endl
			<< "Amax:\n" << Amax.transpose() << std::endl << std::endl;

	// Convert back to Maxon-specific units.
	std::cerr
			<< "Vnew:\n" << Vnew.transpose() << std::endl
			<< "Anew:\n" << Anew.transpose() << std::endl
			<< "Dnew:\n" << Dnew.transpose() << std::endl << std::endl;

	std::cout << t2 << std::endl;

	for(unsigned int i = 0; i < 6; ++i) {
		assert(Vnew(i) <= Vmax[i]);
		assert(Anew(i) <= Amax[i]);
		assert(Dnew(i) <= Amax[i]);
	}

	return 0;
}
