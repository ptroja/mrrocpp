/*!
 * @file
 * @brief File containing methods of the IRp-6ot two fingered gripper kinematic model.
 *
 * @author yoyek
 * @author tkornuta
 * @date Jun 21, 2010
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS irp6ot_tfg
 */

#include "base/lib/com_buf.h"
#include "robot/irp6ot_tfg/kinematic_model_irp6ot_tfg.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot_tfg {

model::model(void)
{
	// Ustawienie etykiety modelu kinematycznego.
	set_kinematic_model_label("Switching to standard kinematic model");

	// Ustawienie parametrow kinematycznych.
	set_kinematic_parameters();

}

void model::set_kinematic_parameters(void)
{

	dir_a_7 = -0.00000000283130;
	dir_b_7 = 0.00001451910074;
	dir_c_7 = 0.074;
	inv_a_7 = 0.3531946456e-5;
	inv_b_7 = 0.2622172716e19;
	inv_c_7 = -0.2831300000e20;
	inv_d_7 = -2564.034320;
	//	gear = 0.0;
	//	theta = 0.000000e+00;

	synchro_motor_position = 4830; // chwytak [-]

	synchro_joint_position = synchro_motor_position;// - gear * theta;

	/* -----------------------------------------------------------------------
	 Zakresy ruchu walow silnikow w radianach.
	 ------------------------------------------------------------------------- */
	lower_limit_axis = -2000;
	upper_limit_axis = 5000;

	/* -----------------------------------------------------------------------
	 Zakresy ruchu poszczegolnych stopni swobody (w radianach lub milimetrach).
	 ------------------------------------------------------------------------- */
	lower_limit_joint = 0.053;
	upper_limit_joint = 0.091;

} //: set_kinematic_parameters


} // namespace irp6ot_tfg
} // namespace kinematic
} // namespace mrrocpp

