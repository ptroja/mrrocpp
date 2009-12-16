// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (lib::VSP)
// Plik:            vsp_m_nint.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Interaktywna powloka procesow VSP
//
// 	-	interaktywne odczytywanie stanu czujnika rzeczywistego, oczekiwanie na zakonczenie operacji
// 	-	operacje read-write + devctl->(write, read, rw)
// 	-	jednowatkowy
//
// Autor:		tkornuta
// Data:		30.11.2006
// ------------------------------------------------------------------------

#include "lib/srlib.h"

#include "vsp/vsp_sensor.h"				// zawiera deklaracje klasy vsp_sensor + struktury komunikacyjne

namespace mrrocpp {
namespace vsp {
namespace sensor {

sensor::sensor (lib::configurator &_config) :
	config(_config),
	is_sensor_configured(false),
	is_reading_ready(false)
{
	/* Lokalizacja procesu wyswietlania komunikatow SR */
	sr_msg = new lib::sr_vsp(lib::VSP,
			config.return_string_value("resourceman_attach_point").c_str(),
			config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION).c_str());

	sr_msg->message ("Communication with SR ready");

	mrrocpp_network_path = config.return_mrrocpp_network_path();
}


void sensor::wait_for_event(void)
{
}

sensor::~sensor() {
	sr_msg->message("VSP terminated");
	delete sr_msg;
}

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

