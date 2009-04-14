// ------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP)
// Plik:			ecp_fs.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody czujnika sily - po stronie procesu ECP.
// Autor:		tkornuta
// Data:		04.11.2005
// ------------------------------------------------------------------------

#include "lib/com_buf.h"

// Zawiera klase bazowa ecp_mp_sensor.
#include "ecp_mp/ecp_mp_sensor.h"
// Zawiera klase ecp_mp_force_sensor.
#include "ecp_mp/ecp_mp_s_force.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
force::force (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task:: base& _ecp_mp_object):
		base (_sensor_name, _section_name, _ecp_mp_object)
{
	union_size = sizeof(image.sensor_union.force);
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
