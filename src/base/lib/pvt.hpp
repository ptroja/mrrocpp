/*!
 * @file pvt.hpp
 * @brief Position-Velocity-Time triples calculation for Maxon EPOS motion controller.
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

#include <stdexcept>

#include <boost/exception/exception.hpp>

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
 * with a trapezoidal velocity profile.
 * @param N number of axes
 * @param Delta destination position delta vector
 * @param Vmax velocity limit vector
 * @param Amax acceleration limit vector
 * @param Dmax deceleration limit vector
 * @param Vnew corrected velocity limit vector
 * @param Anew corrected acceleration limit vector
 * @param Dnew corrected deceleration limit vector
 * @param requestedMotionTime allowed/requested time for motion (if positive)
 * @return time to execute the motion
 */
template <unsigned int N, typename T>
double ppm(
		const Matrix<T,N,1> & Delta,
		const Matrix<T,N,1> & Vmax,
		const Matrix<T,N,1> & Amax,
		Matrix<T,N,1> & Vnew,
		Matrix<T,N,1> & Anew,
		Matrix<T,N,1> & Dnew,
		T requestedMotionTime = 0
		)
{
	Matrix<T,N,3> Times;

	// Iterate over axes
	for(unsigned int l = 0; l < N; ++l) {
		const T delta = Delta(l,0),	// motor position delta
			vmax = Vmax(l,0),				// maximal velocity
			amax = Amax(l,0),				// maximal acceleration
			dmax = -Amax(l,0);				// maximal deceleration

		// Velocity value, when the velocity profile is triangular (eq. 3.32)
		const T VTriangle = amax*std::sqrt(2*delta*dmax/(amax*(dmax-amax)));
		//const T VTriangle = std::sqrt(2*delta*amax*dmax/(dmax-amax));

#if 0
		std::cout << "VTriangle(" << l << ") = " << VTriangle << std::endl;
#endif

		const bool TriangularProfile = (VTriangle <= vmax);

		if(TriangularProfile) {
			// tt: total motion time (eq. 3.33)
			Times(l,2) = std::sqrt(2*delta*(dmax-amax)/(amax*dmax));
			// acceleration and deceleration phase treated as a half of the total motion time
			Times(l,0) = Times(l,1) = Times(l,2)/2;
		} else {
			// ta: time to stop accelerate (eq. 3.35)
			Times(l,0) = vmax/amax;

			// td: time to start deceleration (eq. 3.42)
			Times(l,1) = delta/vmax + vmax/(2*amax) + vmax/(2*dmax);

			// Numerically stable version:
			//Time(l,1) = (2*delta*amax*dmax+vmax*vmax*amax+vmax*vmax*dmax)/(2*vmax*amax*dmax);

			// tt: total motion time (eq. 3.40)
			Times(l,2) = delta/vmax + vmax/(2*amax) - vmax/(2*dmax);

			// Numerically stable version:
			//Time(l,2) = (2*delta*amax*dmax+vmax*vmax*dmax-vmax*vmax*amax)/(2*vmax*amax*dmax);
		}

//		std::cerr << "VLimit[" << l << "]: " << VTriangle <<
//				" => " << (TriangularProfile ? "triangular" : "trapezoidal") << std::endl <<
//				"Time[" << l << "]: " << Times(l,0) << " " << Times(l,1) << " " << Times(l,2) <<
//				std::endl;
	}

	Matrix<T,1,3> maxTimes = Times.colwise().maxCoeff();

//	std::cerr << "maxTimes: " << maxTimes << std::endl;

	// max of acceleration intervals
	const T ta = maxTimes(0);

	// max of constant velocity intervals
	const T tV = (Times.col(1)-Times.col(0)).maxCoeff();

	// max of deceleration intervals
	const T tD = (Times.col(2)-Times.col(1)).maxCoeff();

	// deceleration interval
	const T td = ta + tV;

	T tt = ta + tV + tD;

	if (ta > td) {
		tt += (ta - td);
	}

	// Make the motion longer
	if(requestedMotionTime > 0) {
		if(tt > requestedMotionTime) {
			throw std::runtime_error("requested motion time too short");
		}
		tt = requestedMotionTime;
	}

//	std::cout
//		<< "ta: " << ta << "\t"
//		<< "td: " << td << "\t"
//		<< "tt: " << tt
//		<< std::endl;

	// If the total time is zero there will be no motion
	if(tt > 0) {

		// I guess this can be implemented as a single matrix calculation
		for(unsigned int l = 0; l < N; ++l) {
			const T delta = Delta(l,0);

			// Calculate new parameters if there is motion along an axis
			if(delta) {
				Anew(l,0) = 2*delta/(ta*(tt+td-ta));
				Dnew(l,0) = - (-2*delta/((tt+td-ta)*(tt-td))); // deceleration value (without sign)
				//Dnew(l,0) = (2*delta)/(tt*tt-tt*ta+td*ta-td*td); // Numerically stable (?) version
				Vnew(l,0) = Anew(l,0)*ta;
				//Vnew(l,0) = (2*delta)/(tt+td-ta); // Numerically stable (?) version
			} else {
				Anew(l,0) = Amax(l,0);
				Dnew(l,0) = Amax(l,0);
				Vnew(l,0) = Vmax(l,0);
			}
		}
	} else {
		Vnew = Vmax;
		Anew = Dnew = Amax;
	}

	// These assertions fail because of floating point inequalities
	//assert(Dnew(l,0)<=Amax(l,0));
	//assert(Anew(l,0)<=Amax(l,0));
	//assert(Vnew(l,0)<=Vmax(l,0));;


//	std::cerr <<
//		"Vnew:\n" << Vnew << std::endl <<
//		"Anew:\n" << Anew << std::endl <<
//		"Dnew:\n" << Dnew << std::endl <<
//		std::endl;

//	std::cerr << "tt: " << tt << std::endl;

	return tt;
}

#endif /* PVT_HPP_ */
