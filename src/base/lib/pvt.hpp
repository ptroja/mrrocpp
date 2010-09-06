/*!
 * @file pvt.hpp
 * @brief Position-Velocity-Time triples calculation for Maxon EPOS motino controller.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */
#ifndef PVT_HPP_
#define PVT_HPP_

#include <Eigen/Core>
#include <Eigen/Array>
#include <cmath>
#include <iostream>
#include <cassert>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

/**
 * Determine maximal time to execute a motion in PVT mode
 * @param N number of axes
 * @param Delta destination position delta vector
 * @param Vmax velocity limit vector
 * @param Amax acceleration limit vector
 * @return time to execute the motion
 */
template <unsigned int N>
double pvt(
		const Matrix<double,N,1> & Delta,
		const Matrix<double,N,1> & Vmax,
		const Matrix<double,N,1> & Amax
		)
{
	// value determining which variant to use
	const Matrix<double,N,1> VariantLimit = (8/3) * ((Vmax.cwise() * Vmax).cwise() / Amax).cwise().abs();

	// The motor has enough power (i.e., the acceleration is so high) and the distance to
	// be traversed is long enough for the velocity to exceed Vmax
	const Matrix<bool,N,1> Variant1 = (VariantLimit.cwise() > Delta.cwise().abs());

	Matrix<double,N,1> Time;

	for(unsigned int l = 0; l < N; ++l) {
		if (Variant1(l,0)) {
			// Variant 1
			Time(l,0) = (3/2)*(std::abs(Delta(l,0)/Vmax(l,0)));
		} else {
			// Variant 2
			Time(l,0) = std::sqrt((6)*(std::abs(Delta(l,0)/Amax(l,0))));
		}
	}
//	std::cout
//		<< "VariantLimit:\n" << VariantLimit << std::endl
//		<< "Variant1:\n" << Variant1 << std::endl
//		<< "Time:\n" << Time << std::endl;

	// return the maximum of all axes times
	return Time.maxCoeff();
}

/**
 * Determine maximal time to execute a motion in Profile Position Mode
 * @param N number of axes
 * @param Delta destination position delta vector
 * @param Vmax velocity limit vector
 * @param Amax acceleration limit vector
 * @param Dmax deceleration limit vector
 * @param Vnew corrected velocity limit vector
 * @param Anew corrected acceleration limit vector
 * @param Dnew corrected deceleration limit vector
 * @return time to execute the motion
 */
template <unsigned int N>
double ppm(
		const Matrix<double,N,1> & Delta,
		const Matrix<double,N,1> & Vmax,
		const Matrix<double,N,1> & Amax,
		Matrix<double,N,1> & Vnew,
		Matrix<double,N,1> & Anew,
		Matrix<double,N,1> & Dnew
		)
{
	Matrix<double,N,3> Time;

	for(unsigned int l = 0; l < N; ++l) {
		const double delta = Delta(l,0),
			vmax = Vmax(l,0),
			amax = Amax(l,0),
			dmax = -Amax(l,0);

		const double VTriangle = amax*std::sqrt(2*delta*dmax/(amax*(dmax-amax)));

		std::cout << "VLimit: " << VTriangle << std::endl;

		const bool TriangularProfile = (VTriangle <= vmax);

		if(TriangularProfile) {
			// acceleration and deceleration phase treated as 0
			Time(l,0) = Time(l,1) = 0.0;
			Time(l,2) = std::sqrt(2*delta*(dmax-amax)/(amax*dmax));
		} else {
			// ta: time to stop accelerate
			Time(l,0) = vmax/amax;

			// td: time to start deceleration
			Time(l,1) = delta/vmax + vmax/(2*amax) + vmax/(2*dmax);

			// tt: total motion time
			Time(l,2) = delta/vmax + vmax/(2*amax) - vmax/(2*dmax);
		}
	}

	Matrix<double,1,3> maxTime = Time.colwise().maxCoeff();

	std::cout << "maxTime: " << maxTime << std::endl;
	std::cout << "(maxTime.col(2)-maxTime.col(1): " << (maxTime.col(2)-maxTime.col(1)) << std::endl;

	// total time
	double tt = maxTime.col(2).maxCoeff();
	// acceleration iterval
	const double ta = maxTime.col(0).maxCoeff();
	// deceleration interval
	const double td = tt - (maxTime.col(2)-maxTime.col(1)).maxCoeff();

	if (ta > td) {
		tt += (ta - td);
	}

	std::cout
		<< "ta: " << ta << "\t"
		<< "td: " << td << "\t"
		<< "tt: " << tt
		<< std::endl;

	for(unsigned int l = 0; l < N; ++l) {
		const double delta = Delta(l,0);

		Anew(l,0) = 2*delta/(ta*(tt+td-ta));
		Dnew(l,0) = -2*delta/((tt+td-ta)*(tt-td));
		Vnew(l,0) = Anew(l,0)*ta;

//		assert(Anew(l,0)<Amax(l,0));
	}

	std::cout <<
		"Anew:\n" << Anew << std::endl <<
		"Dnew:\n" << Dnew << std::endl <<
		"Vnew:\n" << Vnew << std::endl <<
		std::endl;

	return tt;
}

#endif /* PVT_HPP_ */
