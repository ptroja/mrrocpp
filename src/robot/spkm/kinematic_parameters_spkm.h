/*!
 * @file
 * @brief File containing the declaration of kinematic_parameters class.
 *
 * @author Tomasz Kornuta
 * @date Jan 5, 2010
 *
 * @ingroup SIF_KINEMATICS spkm
 */

#ifndef KINEMATIC_PARAMETERS_SPKM_H_
#define KINEMATIC_PARAMETERS_SPKM_H_

#include <eigen2/Eigen/Core>
#include <eigen2/Eigen/Geometry>
#include <eigen2/Eigen/LU>

#include "dp_spkm.h"

using namespace Eigen;

namespace mrrocpp {
namespace kinematics {
namespace spkm {

//! Type used for representation of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
typedef Eigen::Matrix<double, 5, 1> Vector5d;

//! Type used for representation of 3-dimensional homogeneous matrices (4x4 doubles).
typedef Eigen::Matrix<double, 4 , 4> Homog4d;

/*!
 * @struct kinematic_parameters_spkm
 * @brief Class storing parameters for PKM and spherical wrist attached to it.
 *
 * @author Tomasz Kornuta
 * @date Jan 05, 2010
 *
 * @ingroup SIF_KINEMATICS
 */
struct kinematic_parameters_spkm {
public:
	//! Constructor - sets the values of the SPKM geometric parameters.
//	kinematic_parameters_spkm();

	//! Lower platform: jb coordinate of P1A in O(ib,jb,kb).
	static const double lA;

	//! Lower platform: ib coordinate of P1B in O(ib,jb,kb).
	static const double lB;

	//! Lower platform: jb coordinate of P1C in O(ib,jb,kb).
	static const double lC;

	//! Upper platform: j coordinate of P4A in P(ijk).
	static const double uA;

	//! Upper platform: i coordinate of P5B in P(ijk).
	static const double uB;

	//! Upper platform: j coordinate of P4C in P(ijk).
	static const double uC;

    //! Vector representing a translation from P (middle of upper P platform) and S (middle of the spherical wrist). An equivalent of <Hx,0,Hz>.
	static const Vector3d P_S_P;

    //! Transformation from P (middle of upper P platform) and S (middle of the spherical wrist).
	static const Homog4d P_S_T;

    //! Transformation from W (SW end-effector) to S (middle of the spherical wrist).
    static const Homog4d W_S_T;

	//! Parameters describing the synchronization positions (in joints).
	static const double synchro_positions[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Parameters related to conversion from motor positions to joints.
	static const double mp2i_ratios[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Encoder resolution.
	static const uint32_t encoder_resolution[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Largest values that motors can reach.
	static const int32_t upper_motor_pos_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Smallest values that motors can reach.
	static const int32_t lower_motor_pos_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Largest values that joints can reach.
	static const double upper_joints_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Smallest values that joints can reach.
	static const double lower_joints_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Largest values of upper thyk alpha angle.
	static const double upper_alpha_thyk_angle_limit[3];

	//! Smallest values of lower thyk alpha angle.
	static const double lower_alpha_thyk_angle_limit[3];

	//! Largest values of upper thyk beta angle.
	static const double upper_beta_thyk_angle_limit[3];

	//! Smallest values of lower thyk beta angle.
	static const double lower_beta_thyk_angle_limit[3];


	// You must overload "operator new" so that it generates 16-bytes-aligned pointers.
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


#endif /* KINEMATIC_PARAMETERS_H_ */
