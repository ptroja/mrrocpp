// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_calibrated_irp6ot_with_wrist.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na torze
//				- deklaracja klasy
//				- wykorzystanie nowego stopnia swobody  jako czynnego stopnia swobody
//				- tor jest biernym stopniem swobody
//				- parametry obliczone zostaly podczas kalibracji
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6OT_KIN_MODEL_WITH_WRIST_CALIBRATED)
#define _IRP6OT_KIN_MODEL_WITH_WRIST_CALIBRATED

// Definicja klasy kinematic_model_irp6ot_with_wrist.
#include "kinematics/irp6_on_track/kinematic_model_irp6ot_with_wrist.h"

namespace mrrocpp {
namespace kinematic {
namespace irp6ot {

class kinematic_model_calibrated_irp6ot_with_wrist: public kinematic_model_irp6ot_with_wrist
{
  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void);

public:
  // Konstruktor.
  kinematic_model_calibrated_irp6ot_with_wrist ( void );

};//: kinematic_model_calibrated_irp6ot_with_wrist;


} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp


#endif
