// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (VSP)
// Plik:			vsp_digital_scales_sensor.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Deklaracja klasy vsp_digital_scales_sensor - odczyt z czujnika zlozonego z linialow.
// Autor:		tkornuta
// Data:		30.11.2006
// ------------------------------------------------------------------------

#if !defined(_VSP_DIGITAL_SCALES_SENSOR_H)
#define _VSP_DIGITAL_SCALES_SENSOR_H

#include "vsp/vsp_sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********** klasa czujnikow po stronie VSP **************/
class digital_scales : public base {
private:
    // Odczyty w pozycji konfiguracji czujnika.
    double position_zero[6];
    // Liczba linialow.
    short number_of_scales;
    // Flagi stanu procesu.
    short readings_initiated;
    
public:
    // Konstruktor czujnika wirtualnego.
    digital_scales(configurator &_config);
    // Destruktor czujnika wirtualnego.
    ~digital_scales(void);
    // Konfiguracja czujnika.
    void configure_sensor (void);
    // Inicjacja odczytu.
    void initiate_reading (void);
    // Odeslanie odczytu.
    void get_reading (void);
}; // end: class vsp_ds_sensor

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
