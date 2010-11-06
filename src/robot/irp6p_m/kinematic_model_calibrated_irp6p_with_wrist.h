/*!
 * @file
 * @brief File containing the IRp-6p with wrist (6DOFs) calibrated kinematic model class.
 *
 * @author tkornuta
 * @date 14.02.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS irp6p_m
 */

#if !defined(_IRP6P_KIN_MODEL_WITH_WRIST_CALIBRATED)
#define _IRP6P_KIN_MODEL_WITH_WRIST_CALIBRATED

#include "robot/irp6p_m/kinematic_model_irp6p_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6p {

/*!
 *
 * @brief The kinematic model utilizes six IRP-6p DOFs.
 *
 * The model_with_wrist kinematic model utilizes six (with an additional one in the wrist) IRP-6p DOFs.
 * Parameters were computed in the calibration process.
 *
 * @author tkornuta
 * @date 14.02.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS
 */
class model_calibrated_with_wrist : public model_with_wrist
{
	/**
	 * @brief Method responsible for kinematic parameters setting.
	 *
	 * Those parameters were computed during the calibration process.
	 */
	virtual void set_kinematic_parameters(void);

public:
	/**
	 * @brief Constructor.
	 * @param _number_of_servos Number of servos (joints).
	 */
	model_calibrated_with_wrist(int _number_of_servos);

};

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp

#endif
