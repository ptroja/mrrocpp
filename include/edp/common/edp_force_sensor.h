// -------------------------------------------------------------------------
//                            vsp.h		dla QNX6.2
//
// Definicje klasy edp_force_sensor
//
// Ostatnia modyfikacja: 2005
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_FORCE_SENSOR_H)
#define _EDP_FORCE_SENSOR_H

#include "lib/ForceTrans.h"
#include "common/sensor.h"				// klasa bazowa sensor
#include "edp/common/edp.h"				// klasa bazowa sensor

#ifdef __cplusplus
extern "C"
{
#endif

    /********** klasa czujnikow po stronie EDP **************/
    class edp_force_sensor : public sensor
    {

    protected:

        short is_sensor_configured;		// czy czujnik skonfigurowany?
        short is_reading_ready;			// czy jakikolwiek odczyt jest gotowy?

        ForceTrans *gravity_transformation; // klasa likwidujaca wplyw grawitacji na czujnik

    public:

        edp_irp6s_postument_track_effector &master;
        edp_force_sensor(edp_irp6s_postument_track_effector &_master);

        virtual void wait_for_event(void);			// oczekiwanie na zdarzenie

    }
    ; // end: class edp_force_sensor


    // Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
    edp_force_sensor* return_created_edp_force_sensor (edp_irp6s_postument_track_effector &_master);


#ifdef __cplusplus
};
#endif
#endif
