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
	for(double q = 1;; q *= 10.0) {
	std::cout << "*** q = " << q << std::endl;
	Matrix <double, 6, 1> Delta, Vmax, Amax, Vnew, Anew, Dnew;

	Matrix <double, 6, 1> Vdefault, Adefault, Ddefault;

	// Move in [qc]
	Delta(0) = 0;
	Delta(1) = 0;
	Delta(2) = 0;
	Delta(3) = 0;
	Delta(4) = 0;
	Delta(5) = q;

	for(int i = 0; i < 6; ++i) {
		Vdefault(i) = _Vdefault[i];
		Adefault(i) = _Adefault[i];
		Ddefault(i) = _Ddefault[i];
		Delta(i) = Delta(i) / encoder_resolution[i]; // [rot]
		Vmax(i) = Vdefault(i) / 60; // [rpm]
		Amax(i) = Adefault(i) / 60; // [rpm/s]
	}

	const double t2 = ppm<6>(Delta, Vmax, Amax, Vnew, Anew, Dnew);

	Vnew *= 60;
	Anew *= 60;
	Dnew *= 60;

	for(unsigned int i = 0; i < 6; ++i) {
		Vnew(i) = trunc(Vnew(i));
		Anew(i) = trunc(Anew(i));
		Dnew(i) = trunc(Dnew(i));
	}

	std::cerr <<
		"Vdiff:\n" << std::dec << Vdefault-Vnew << std::endl <<
		"Adiff:\n" << std::scientific << Adefault-Anew << std::endl <<
		"Ddiff:\n" << std::scientific << Ddefault-Dnew << std::endl <<
		std::endl;

	for(unsigned int i = 0; i < 6; ++i) {
		assert(Vnew(i) <= Vdefault[i]);
		assert(Anew(i) <= Adefault[i]);
		assert(Dnew(i) <= Ddefault[i]);
	}

	std::cerr << std::fixed << "ppm: " << t2 << std::endl;
	}

	return 0;
}
