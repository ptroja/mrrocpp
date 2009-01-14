// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_calibrated_irp6p_with_wrist.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- deklaracja klasy
//				- wykorzystanie nowego stopnia swobody  jako czynnego stopnia swobody
//				- parametry obliczone zostaly podczas kalibracji
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6P_KIN_MODEL_WITH_WRIST_CALIBRATED)  
#define _IRP6P_KIN_MODEL_WITH_WRIST_CALIBRATED

// Definicja typu frame_tab.
#include "common/impconst.h"
// Definicja klasy kinematic_model.
#include "kinematics/irp6_postument/kinematic_model_irp6p_with_wrist.h" 

class kinematic_model_calibrated_irp6p_with_wrist: public kinematic_model_irp6p_with_wrist
{
  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void);
public:
  // Konstruktor.
  kinematic_model_calibrated_irp6p_with_wrist (void);

};//: kinematic_model_calibrated_irp6p_with_wrist

#endif					   

