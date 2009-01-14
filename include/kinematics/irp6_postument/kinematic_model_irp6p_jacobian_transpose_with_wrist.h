// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6p_jacobian_transpose_with_wrist.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- deklaracja klasy
//				- przedefiniowanie rozwiazania odwrotnego zadania 
//				  kinematyki - metoda uwzgledniajaca jakobian transponowany
//
// Autor:		Anna Maria Sibilska
// Data:		18.07.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6P_KIN_MODEL_WITH_WRIST_JACOBIAN_TRANSPOSE)  
#define _IRP6P_KIN_MODEL_WITH_WRIST_JACOBIAN_TRANSPOSE

// Definicja typu frame_tab.
#include "common/impconst.h"
// Definicja klasy kinematic_model.
#include "edp/irp6_postument/kinematic_model_irp6p_with_wrist.h" 

class kinematic_model_irp6p_jacobian_transpose_with_wrist: public kinematic_model_irp6p_with_wrist
{

public:
  // Konstruktor.
  kinematic_model_irp6p_jacobian_transpose_with_wrist (void);

  //Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, frame_tab* local_desired_end_effector_frame);

};//: kinematic_model_irp6p_jacobian_transpose_with_wrist

#endif					   
