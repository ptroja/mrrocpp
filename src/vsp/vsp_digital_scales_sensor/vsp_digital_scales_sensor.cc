// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (lib::VSP)
// Plik:            vsp_dss.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		definicje metod klasy vsp_dsensor
// Autor:		tkornuta
// Data:		30.11.2006
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <pthread.h>            // pthread_barrier_t

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"

// Konfigurator
#include "lib/configurator.h"
#include "vsp/vsp_digital_scales_sensor/vsp_digital_scales_sensor.h"
#include "vsp/vsp_digital_scales_sensor/moxaclass.h"

namespace mrrocpp {
namespace vsp {
namespace common {

// Czujnik wirtualny
extern sensor::digital_scales *vs;

}

namespace sensor {
/********************************* GLOBALS **********************************/


// Bariera uzywana do zawieszania watkow w oczekiwaniu na polecenie INITIATE_READING.
static pthread_barrier_t initiate_reading_barrier;
// Bariera uzywana przez watek koordynatora - oczekiwanie na odczyty GET_READING.
static pthread_barrier_t reading_ready_barrier;

/*************************** DIGITAL SCALE THREAD ****************************/
void* digital_scale_thread(void * arg){
    // Odczytanie numeru linialu.
    int number = (int) arg;
    // Ustawienie priorytetu watku.
    lib::set_thread_priority(pthread_self() , MAX_PRIORITY-4);
    // Stworzenie obiektu pomiarowego zwiazanego z danym linialem.
    MOXADigitalScale ds(number);
    // Glowna petla oczekiwania na polecenie odczytu.
    while(true){
        // Oczekiwanie na polecenie (zawieszenie na barierze).
        pthread_barrier_wait( &initiate_reading_barrier);
        // Koniec pracy.
        pthread_testcancel();

        try{
            // Pobranie odczytu.
            ds.get_reading();
            // Przeksztalcenie odczytu do postaci zmiennoprzecinkowej.
            common::vs->image.sensor_union.ds.readings[number-1] = ds.transform_reading_to_double();
            } // end: try
        catch(lib::sensor::sensor_error e){
        	common::vs->sr_msg->message(e.error_class, e.error_no);
        	common::vs->image.sensor_union.ds.readings[number-1] = 0;
            } // end: catch
        // Odczyt gotowy (zawieszenie na barierze).
        pthread_barrier_wait( &reading_ready_barrier);
        }

        return NULL;
    } // end: digital_scale_thread

/*****************************  KONSTRUKTOR *********************************/
digital_scales::digital_scales(lib::configurator &_config)  : sensor(_config) {
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.ds);

    // Struktura do pobierania danych z pliku konfiguracyjnego.
    number_of_scales = config.return_int_value("number_of_scales");

    // Jesii za duzo linialow.
    if(number_of_scales > 6)
        number_of_scales = 6;
    // Jesii za malo linialow.
    if(number_of_scales < 0)
        number_of_scales = 0;
    // Stworzenie barier uzywanych do synchronizacji watkow.
    pthread_barrier_init(&initiate_reading_barrier, NULL, number_of_scales+1);
    pthread_barrier_init(&reading_ready_barrier, NULL, number_of_scales+1);

    // Inicjacja watkow.
    for(int i=1; i<= number_of_scales; i++)
        pthread_create( NULL, NULL, &digital_scale_thread, (void *)i);
    // Zerowe polozenia poczatkowe.
    for(int i=0; i<6; i++){
        position_zero[i] = 0;
        image.sensor_union.ds.readings[i]=0;
        }

    // Ustawienie flagi stanu procesu.
    readings_initiated = false;
    }


digital_scales::~digital_scales(void){
	// TODO: call pthread_cancel() on all created threads
	// TODO: and then do pthread_join() on them
	pthread_barrier_destroy(&reading_ready_barrier);
	pthread_barrier_destroy(&initiate_reading_barrier);
}


/************************** CONFIGURE SENSOR ******************************/
void digital_scales::configure_sensor (void){
    // Odblokowanie watkow, jezeli odczyty byly juz zainicjowane.
    if (readings_initiated)
        pthread_barrier_wait(&reading_ready_barrier);
    // Rozpoczecie pomiarow.
    pthread_barrier_wait(&initiate_reading_barrier);
    // Oczekiwanie na zakonczenie pomiarow.
    pthread_barrier_wait(&reading_ready_barrier);
    // Przepisanie obecnych odczytow jako polozania zerowego.
    for(int i=0; i<number_of_scales; i++)
        position_zero[i] = image.sensor_union.ds.readings[i];
    // Ustawienie flagi stanu procesu.
    readings_initiated = false;
    common::vs->sr_msg->message ("Digital Scale sensor calibrated");
    }

/**************************** INITIATE READING *******************************/
void digital_scales::initiate_reading (void){
    // Odblokowanie watkow, jezeli odczyty byly juz zainicjowane.
    if (readings_initiated)
        pthread_barrier_wait(&reading_ready_barrier);
    // Ustawienie flagi stanu procesu.
    readings_initiated = true;
    // Rozpoczecie pomiarow.
    pthread_barrier_wait(&initiate_reading_barrier);
    }

/***************************** GET  READING *********************************/
void digital_scales::get_reading (void){
    // Sprawdzenie, czy odczyty sa zainicjowane.
    if (!readings_initiated)
        throw sensor_error (lib::NON_FATAL_ERROR, READING_NOT_READY);
    // Oczekiwanie na zakonczenie pomiarow.
    pthread_barrier_wait(&reading_ready_barrier);
    // Odczyty sa gotowe.
    from_vsp.vsp_report= lib::VSP_REPLY_OK;
    // Przepisanie odczytow do bufora komunikacyjnego.
    for(int i=0; i<6; i++)
        if(i<number_of_scales){
            // odczyt = obraz czujnika - polozenie zerowe
            from_vsp.comm_image.sensor_union.ds.readings[i] = image.sensor_union.ds.readings[i] - position_zero[i];
        }else
            from_vsp.comm_image.sensor_union.ds.readings[i] = 0;
    // Ustawienie flagi stanu procesu.
    readings_initiated = false;
    }

VSP_CREATE_SENSOR(digital_scales)

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

