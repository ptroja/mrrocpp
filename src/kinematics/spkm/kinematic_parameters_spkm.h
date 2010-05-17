/*!
 * \file kinematic_parameters_spkm.h
 * \brief File containing the declaration of kinematic_parameters class.
 *
 * \author tkornuta
 * \date Jan 5, 2010
 */

#ifndef KINEMATIC_PARAMETERS_SPKM_H_
#define KINEMATIC_PARAMETERS_SPKM_H_

// Libraries stddef and stdlib are required by Eigen to compile with QCC.
#include <stddef.h>
#include <stdlib.h>
#include <eigen2/Eigen/Core>
#include <eigen2/Eigen/Geometry>
#include <eigen2/Eigen/LU>

using namespace Eigen;

namespace mrrocpp {
namespace kinematics {
namespace spkm {

//! Type used for representation of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
typedef Matrix<double, 5, 1> Vector5d;

//! Type used for representation of 3-dimensional homogeneous matrices (4x4 doubles).
typedef Transform<double, 3> Homog4d;

/*!
 * \struct kinematic_parameters_spkm
 * \brief Class storing parameters for PKM and spherical wrist attached to it.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */
struct kinematic_parameters_spkm {
public:
	//! Constructor - sets the values of the SPKM geometric parameters.
	kinematic_parameters_spkm();

	//! Vector representing a translation from W (end of the spherical wrist) to S (middle of the spherical wrist). An equivalent of <Sx,Sy,Sz>.
	Vector3d W_S_P;

	//! Matrix computed on the base of W_S_P (with identity rotation matrix) - a transformation from end of SW (W) to its middle (S).
	Homog4d W_S_T;

	//! Vector representing a translation from P (middle of upper PM platform) to S (middle of the spherical wrist). An equivalent of <Hx,0,Hz>.
	Vector3d S_P_P;

	//! Working mode parameter for leg A (equal to +-1).
	short delta_A;

	//! Working mode parameter for leg C (equal to +-1).
	short delta_C;

	//! First working mode parameter for leg B (equal to +-1).
	short delta_B1;

	//! Second working mode parameter for leg A (equal to +-1).
	short delta_B2;

	//! jb coordinate of P1A in O(ib,jb,kb).
	double dA;

	//! ib coordinate of P1B in O(ib,jb,kb).
	double dB;

	//! jb coordinate of P1C in O(ib,jb,kb).
	double dC;

	//! j coordinate of P4A in P(ijk).
	double pA;

	//! i coordinate of P5B in P(ijk).
	double pB;

	//! j coordinate of P4C in P(ijk).
	double pC;

	//! Distance between P1A and P2A along with pi_alpha.
	double l12A;

	//! Distance between P1C and P2C along with pi_alpha.
	double l12C;

	//! Distance between P4A and pi_h, according to e_h.
	double hA;

	//! Distance between P4C and pi_h, according to e_h.
	double hC;
};

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp

#endif /* KINEMATIC_PARAMETERS_H_ */
