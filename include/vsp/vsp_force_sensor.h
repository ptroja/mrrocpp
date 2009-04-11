// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (VSP) 
// Plik:			vsp_force_sensor.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Deklaracja klasy vsp_force_sensor - czujnik sily.
// Autor:		tkornuta
// Data:		09.11.2005
// -------------------------------------------------------------------------

#if !defined(_VSP_FORCE_SENSOR_H)
#define _VSP_FORCE_SENSOR_H

#include "vsp/vsp_sensor.h"



/********** klasa czujnikow po stronie VSP **************/
class vsp_force_sensor : public vsp_sensor {
private:
	int edp_vsp_fd; // do polaczenia z EDP

	struct sigevent event;
	VSP_EDP_message vsp_edp_command;// by Y do komuniacji z EDP
	EDP_VSP_reply edp_vsp_reply;
	
	unsigned int ms_nr; // numer odczytu z czujnika
	
	struct timespec start[9];

public:
    // Konstruktor czujnika wirtualnego.
	vsp_force_sensor (configurator &_config);

    // Konfiguracja czujnika.
    void configure_sensor (void);
    // Inicjacja odczytu.
    void initiate_reading (void);
    // Oczekiwanie na wydarzenie.
	void wait_for_event(void);
    // Odeslanie odczytu.
    void get_reading (void);
}; // end: class vsp_ds_sensor

#endif

