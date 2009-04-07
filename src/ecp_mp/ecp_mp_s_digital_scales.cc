// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP)
// Plik:			ecp_m_em.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		metody klasy ecp_mp_sensor dla czujnika z linialami
// Autor:		tkornuta
// Data:		30.11.2006
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/

#include "ecp_mp/ecp_mp_s_digital_scales.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
digital_scales::digital_scales(SENSOR_ENUM _sensor_name, const char* _section_name, task:: base& _ecp_mp_object) :
	base(_sensor_name, _section_name, _ecp_mp_object)
{
	// Ustawienie wielkosci przesylanej unii.
	union_size = sizeof(image.sensor_union.ds);
	// Wyzerowanie odczytow.
	for (int i =0; i<6; i++)
		image.sensor_union.ds.readings[i] = 0;
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
