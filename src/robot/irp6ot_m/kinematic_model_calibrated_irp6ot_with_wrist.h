/*!
 * @file
 * @brief File containing the model_calibrated_with_wrist class.
 *
 * The model_calibrated_with_wrist kinematic model utilizes six out of seven IRP-6ot DOF - the newly added one is active, the track is passive.
 * Utilizes calibrated parameters of the kinematics (old calibration, made with POLMAN-6L).
 *
 * @author tkornuta
 * @date 14.02.2007
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS irp6ot_m
 */

#if !defined(_IRP6OT_KIN_MODEL_WITH_WRIST_CALIBRATED)
#define _IRP6OT_KIN_MODEL_WITH_WRIST_CALIBRATED

#include "robot/irp6ot_m/kinematic_model_irp6ot_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot {

/*!
 *
 * @brief The kinematic model utilizes six out of seven IRP-6ot DOF - the newly added one is active, the track is passive.
 *
 * Utilizes calibrated parameters of the kinematics (old calibration, made with POLMAN-6L).
 *
 * @author tkornuta
 * @date 14.02.2007
 *
 * @ingroup KINEMATICS,IRP6OT_KINEMATICS
 */
class model_calibrated_with_wrist : public model_with_wrist
{
	//! Method responsible for kinematic parameters setting.
	virtual void set_kinematic_parameters(void);

public:
	/**
	 * @brief Constructor.
	 * @param _number_of_servos Number of servos (joints).
	 */
	model_calibrated_with_wrist(int _number_of_servos);
};

} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp


#endif
