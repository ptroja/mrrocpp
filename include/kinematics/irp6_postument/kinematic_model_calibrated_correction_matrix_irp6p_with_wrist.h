// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_calibrated_correction_matrix_irp6p_with_wrist.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- deklaracja klasy
//				- wykorzystanie nowego stopnia swobody  jako czynnego stopnia swobody
//				- tor jest biernym stopniem swobody
//				- parametry obliczone zostaly podczas kalibracji
//				- uzycie macierzy korekcji do lokalnej poprawy.
//
// Autor:		tkornuta
// Data:		17.0.3.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6P_KIN_MODEL_WITH_WRIST_CALIBRATED_CORRECTED)  
#define _IRP6P_KIN_MODEL_WITH_WRIST_CALIBRATED_CORRECTED

// Definicja typu frame_tab.
#include "common/impconst.h"
// Definicja klasy kinematic_model_calibrated_irp6p_with_wrist.
#include "edp/irp6_postument/kinematic_model_calibrated_irp6p_with_wrist.h"

class kinematic_model_calibrated_correction_matrix_irp6p_with_wrist: public kinematic_model_calibrated_irp6p_with_wrist
{
protected:
  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void);

public:
  // Konstruktor.
  kinematic_model_calibrated_correction_matrix_irp6p_with_wrist ( void );

};//: kinematic_model_calibrated_correction_matrix_irp6p_with_wrist;

#endif					   

