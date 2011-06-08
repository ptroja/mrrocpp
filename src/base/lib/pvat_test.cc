/*!
 * @file pvat_test.cc
 * @brief Test utility for the
 *
 * @date 22-04-2011
 * @author tkornuta
 *
 * @ingroup LIB
 */

#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <boost/lexical_cast.hpp>

#include <Eigen/Core>

#include "pvat_cartesian.hpp"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

using namespace std;
using namespace mrrocpp::lib::pvat;

int main(int argc, char *argv[])
{
/*
	// Calculate time - currently the motion time is set to 5s.
	// TODO: analyze required (desired) movement time -> III cases: t<t_req, t=t_req, t>t_req.
	double motion_time = 5;

	// Divide motion time into segments (time slices).
	 Eigen::Matrix <double, 10, 1> time_deltas;
	 //pvat_divide_motion_time_into_constant_time_deltas <3> (time_deltas, motion_time);
	 time_deltas<< 0.5, 0.5, 1, 1, 1, 0.2, 0.2, 0.2, 0.2, 0.2;

	 // Interpolate motor poses - equal to number of segments +1 (the start pose).
	 Eigen::Matrix <double, 10+1, 1> motor_interpolations;
	 //pvat_linear_interpolate_motor_poses <3+1, 1> (motor_interpolations, motion_time, time_deltas, get_current_kinematic_model(), desired_joints_old, current_end_effector_frame, desired_end_effector_frame);
	 motor_interpolations << 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0;

	 // Compute motor_deltas for segments.
	 Eigen::Matrix <double, 10, 1> motor_deltas_for_segments;
	 pvat_compute_motor_deltas_for_segments <10, 1> (motor_deltas_for_segments, motor_interpolations);

	 // Compute tau coefficient matrix of the (1.48) equation.
	 Eigen::Matrix <double, 10, 10> tau_coefficients;
	 pvat_compute_tau_coefficients_matrix <10> (tau_coefficients, time_deltas);

	 // Compute right side vector of the (1.48) equation - for all motors!!
	 Eigen::Matrix <double, 10, 1> right_side_coefficients;
	 pvat_compute_right_side_coefficients_vector <10, 1> (right_side_coefficients, motor_deltas_for_segments, time_deltas);

	 // Compute 2w polynomial coefficients for all motors!!
	 Eigen::Matrix <double, 10, 1> motor_2w;
	 pvat_compute_motor_2w_polynomial_coefficients <10, 1> (motor_2w, tau_coefficients, right_side_coefficients);

	 // Compute 1w polynomial coefficients for all motors!!
	 Eigen::Matrix <double, 10, 1> motor_1w;
	 pvat_compute_motor_1w_polynomial_coefficients <10, 1> (motor_1w, motor_2w, motor_deltas_for_segments, time_deltas);

	 // Compute 3w polynomial coefficients for all motors!!
	 Eigen::Matrix <double, 10, 1> motor_3w;
	 pvat_compute_motor_3w_polynomial_coefficients <10, 1> (motor_3w, motor_2w, motor_deltas_for_segments, time_deltas);

	 // Compute 0w polynomial coefficients for all motors!!
	 Eigen::Matrix <double, 10, 1> motor_0w;
	 pvat_compute_motor_0w_polynomial_coefficients <10, 1> (motor_0w, motor_interpolations);

	 cout << "time deltas = [ \n" << time_deltas << "\n ]; \n";
	 cout << "m0w = [\n" << motor_0w <<  "\n ]; \n";
	 cout << "m1w = [\n" << motor_1w <<  "\n ]; \n";
	 cout << "m2w = [\n" << motor_2w <<  "\n ]; \n";
	 cout << "m3w = [\n" << motor_3w <<  "\n ]; \n";
	 */

/*	Matrix<double,2,2> m;
	Matrix<double,2,2> n;
	Matrix<double,2,2> result;

	m << 1, 2, 3, 4;
	n << 5, 6, 7, 8;

	// value determining which variant to use

	//result = m * n;
	//cout << "-- Matrix m*n: --" << endl << result << endl << endl;
	result = (m.cwise() * n);
	cout << "-- cwise m*n: --" << endl << result << endl << endl;
	result = (m.cwise() * m);
	cout << "-- cwise m*m: --" << endl << result << endl << endl;
	result = (m.cwise() * m).cwise() / (3.0 * n);
	cout << "-- cwise (m*m)/n: --" << endl << result << endl << endl;*/


/*
	Eigen::Matrix <double, 2, 1> m;
	Eigen::Matrix <double, 2, 1> n;
	Eigen::Matrix <double, 2, 1> result;

	m << 1, 2;
	n << 5, 6;

	result = m * n;
	cout << "-- Matrix m*n: --" << endl << result << endl << endl;
	result = ((m.cwise() * n).cwise());
	cout << "-- cwise m*n: --" << endl << result << endl << endl;
	result = ((m.cwise() / n).cwise());
	cout << "-- cwise m/n: --" << endl << result << endl << endl;
*/


	struct timeval tv;
	if(gettimeofday(&tv, NULL) == -1) {
		perror("gettimeofday()");
	}

	std::string name;
	name = boost::lexical_cast <std::string>(tv.tv_sec);
	//tv_usec = tv.tv_usec;
	std::cout << name << std::endl;

}
