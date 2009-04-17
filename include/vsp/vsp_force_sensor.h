// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (lib::VSP) 
// Plik:			vsp_force_sensor.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Deklaracja klasy vsp_force_sensor - czujnik sily.
// Autor:		tkornuta
// Data:		09.11.2005
// -------------------------------------------------------------------------

#if !defined(_VSP_FORCE_SENSOR_H)
#define _VSP_FORCE_SENSOR_H

#include "vsp/vsp_sensor.h"


namespace mrrocpp {
namespace vsp {
namespace sensor {


/********** klasa czujnikow po stronie VSP **************/
class force : public sensor {
private:
	int edp_vsp_fd; // do polaczenia z EDP

	struct sigevent event;
	lib::VSP_EDP_message vsp_edp_command;// by Y do komuniacji z EDP
	lib::EDP_VSP_reply edp_vsp_reply;
	
	unsigned int ms_nr; // numer odczytu z czujnika
	
	struct timespec start[9];

public:
    // Konstruktor czujnika wirtualnego.
	force (lib::configurator &_config);

    // Konfiguracja czujnika.
    void configure_sensor (void);
    // Inicjacja odczytu.
    void initiate_reading (void);
    // Oczekiwanie na wydarzenie.
	void wait_for_event(void);
    // Odeslanie odczytu.
    void get_reading (void);
}; // end: class vsp_ds_sensor

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif

