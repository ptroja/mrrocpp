// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (lib::VSP)
// Plik:			vsp_digital_scales_sensor.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Deklaracja klasy vsp_digital_scales_sensor - odczyt z czujnika zlozonego z linialow.
// Autor:		tkornuta
// Data:		30.11.2006
// ------------------------------------------------------------------------

#if !defined(_VSP_PP_SENSOR_H)
#define _VSP_PP_SENSOR_H

#include "vsp/vsp_sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********** klasa czujnikow po stronie VSP **************/
class pp : public sensor{
private:
    // Deskryptor portu RS-232
    int RS_descriptor;
    // Deskryptor pliku konfiguracyjnego joysticka
    int Joy_descriptor;
	// Slowo odebrane / do wyslania
	char Word_received, Word_to_send, Command_received;
    // Polozenia zerowe osi joysticka
    char position_lo_zero[3];
    char position_hi_zero[3];
    // Skrajne wychylenia osi joysticka
    char position_lo_ext[3];
    char position_hi_ext[3];
    // Odczyty osi joysticka
    char axis_reading[3];
    // Obraz czujnika
    double joy_axis_img[3];
    // Flagi stanu procesu.
    short readings_initiated;
    
public:
    // Konstruktor czujnika wirtualnego.
    pp(lib::configurator &_config);
    // Destruktor czujnika wirtualnego.
    ~pp(void);
    // Konfiguracja czujnika.
    void configure_sensor (void);
    // Inicjacja odczytu.
    void initiate_reading (void);
    // Odeslanie odczytu.
    void get_reading (void);
    // Koniec pracy.
    void terminate (void);
}; // end: class vsp_ds_sensor

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
