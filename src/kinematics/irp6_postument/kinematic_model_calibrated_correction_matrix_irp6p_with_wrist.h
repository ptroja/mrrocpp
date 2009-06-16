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

// Definicja klasy kinematic_model_calibrated_irp6p_with_wrist.
#include "kinematics/irp6_postument/kinematic_model_calibrated_irp6p_with_wrist.h"

namespace mrrocpp {
namespace kinematic {
namespace irp6p {

class model_calibrated_correction_matrix_with_wrist: public model_calibrated_with_wrist
{
protected:
  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void);

public:
  // Konstruktor.
  model_calibrated_correction_matrix_with_wrist ( void );

};//: kinematic_model_calibrated_correction_matrix_irp6p_with_wrist;

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp

#endif
