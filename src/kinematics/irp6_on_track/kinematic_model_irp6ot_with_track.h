// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6ot_with_track.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na torze
//				- deklaracja klasy
//				- wykorzystanie toru jako czynnego stopnia swobody
//
// Autor:		tkornuta
// Data:		31.01.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6OT_KIN_MODEL_WITH_TRACK)
#define _IRP6OT_KIN_MODEL_WITH_TRACK

// Definicja klasy kinematic_model.
#include "kinematics/irp6_on_track/kinematic_model_irp6ot_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot {

class model_with_track : public model_with_wrist
{

public:
  // Konstruktor.
  model_with_track (int _number_of_servos);

  // Rozwiazanie prostego zagadnienia kinematyki.
  virtual void direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix & local_current_end_effector_frame);

  // Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

};//: kinematic_model_irp6ot_with_track;

} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp

#endif
