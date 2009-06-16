// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (lib::VSP) 
// Plik:	vsp_time_sensor.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:	Deklaracja klasy vsp_time_sensor - czujnik sily.
// Autor:	ptrojane
// Data:	19.04.2007
// -------------------------------------------------------------------------

#if !defined(_VSP_TIME_SENSOR_H)
#define _VSP_TIME_SENSOR_H

#include "vsp/vsp_sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********** klasa czujnikow po stronie VSP **************/
class time : public sensor {
private:
	
    struct timespec ts;

public:
    // Konstruktor czujnika wirtualnego.
    time (lib::configurator &_config);

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

#endif /* _VSP_TIME_SENSOR_H */
