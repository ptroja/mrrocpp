/*!
 * @file pvt_test.cc
 * @brief Position-Velocity-Time triples calculation for Maxon EPOS motino controller - test utility.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include "pvt.hpp"

#include <Eigen/Core>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

int
main(int argc, char *argv[])
{
	Matrix<double,1,1> Goal, Vmax, Amax, Vnew, Anew, Dnew;

	Goal(0,0) = 2.0; // position delta
	Vmax(0,0) = 1.0; // maximal velotivy
	Amax(0,0) = 1.0; // maximal acceleration

	const double t1 = pvt<1>(Goal, Vmax, Amax);
	const double t2 = ppm<1>(Goal, Vmax, Amax, Vnew, Anew, Dnew);

	printf("pvt: %f\tppm: %f\n", t1, t2);

	return 0;
}
