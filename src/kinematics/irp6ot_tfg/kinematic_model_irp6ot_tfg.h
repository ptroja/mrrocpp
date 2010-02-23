// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_conveyor.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki tasmociogu
//				- deklaracja klasy
//
// Autor:		yoyek
// Data:		31.01.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6OT_TFG_KIN_MODEL)
#define _IRP6OT_TFG_KIN_MODEL

#include "kinematics/common/kinematic_model_irp6_tfg.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot_tfg {

class model: public common::kinematic_model_irp6_tfg
{
protected:

	// Ustawienie parametrow kinematycznych.
	void set_kinematic_parameters(void);

public:
	// Konstruktor.
	model(void);

};//: kinematic_model_conveyor;

} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp

#endif
