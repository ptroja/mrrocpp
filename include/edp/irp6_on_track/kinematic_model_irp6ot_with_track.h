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

// Definicja typu frame_tab.
#include "common/impconst.h"
// Definicja klasy kinematic_model.
#include "edp/irp6_on_track/kinematic_model_irp6ot_with_wrist.h"

class kinematic_model_irp6ot_with_track : public kinematic_model_irp6ot_with_wrist
{

public:
  // Konstruktor.
  kinematic_model_irp6ot_with_track (void);

  // Rozwiazanie prostego zagadnienia kinematyki.
  virtual void direct_kinematics_transform(const double* local_current_joints, frame_tab* local_current_end_effector_frame);
  
  // Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, frame_tab* local_desired_end_effector_frame);

};//: kinematic_model_irp6ot_with_track;

#endif					   

