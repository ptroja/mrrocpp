/*!
 * \file kinematic_parameters_bird_hand.h
 * \brief File containing the declaration of kinematic_parameters class.
 *
 * \author tkornuta
 * \date Jan 5, 2010
 */

#ifndef KINEMATIC_PARAMETERS_BIRD_HAND_H_
#define KINEMATIC_PARAMETERS_BIRD_HAND_H_

namespace mrrocpp {
namespace kinematics {
namespace bird_hand {

/*!
 * \struct kinematic_parameters_bird_hand
 * \brief Class storing parameters.
 *
 * \author kczajkowski
 * \date May 28, 2010
 */
struct kinematic_parameters_bird_hand {
public:
	//! Constructor - sets the values of the BIRD_HAND geometric parameters.
	kinematic_parameters_bird_hand();

	// Zmienne opisujace przekladnie dla wszystkich stopni swobody.
	double gear[8];

	// Zakresy ruchu walow silnikow w radianach.
	double lower_limit_axis[8];
	double upper_limit_axis[8];
	// Zakresy ruchu stopni swobody w radianach.
	double lower_limit_joint[8];
	double upper_limit_joint[8];

	// Polozenia synchronizacji (polozenia walow silnikow).
	double synchro_motor_position[8];
	// Polozenia synchronizacji (polozenia we wspolrzednych wewnetrznych).
	double synchro_joint_position[8];
};

} // namespace bird_hand
} // namespace kinematic
} // namespace mrrocpp

#endif /* KINEMATIC_PARAMETERS_H_ */
