// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (lib::VSP)
// Plik:			vsp_digital_scales_sensor.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Deklaracja klasy vsp_digital_scales_sensor - odczyt z czujnika zlozonego z linialow.
// Autor:		tkornuta
// Data:		30.11.2006
// ------------------------------------------------------------------------

#if !defined(_VSP_DIGITAL_SCALES_SENSOR_H)
#define _VSP_DIGITAL_SCALES_SENSOR_H

#include "vsp/vsp_sensor.h"

#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********** klasa czujnikow po stronie VSP **************/
class digital_scales : public sensor {
private:
    // Odczyty w pozycji konfiguracji czujnika.
    double position_zero[6];
    // Liczba linialow.
    unsigned int number_of_scales;
    // Flagi stanu procesu.
    bool readings_initiated;

    boost::ptr_vector<boost::thread> threads;

    // TODO: the following should be boost::barrier

    // Bariera uzywana do zawieszania watkow w oczekiwaniu na polecenie INITIATE_READING.
    pthread_barrier_t initiate_reading_barrier;
    // Bariera uzywana przez watek koordynatora - oczekiwanie na odczyty GET_READING.
    pthread_barrier_t reading_ready_barrier;

public:
    // Konstruktor czujnika wirtualnego.
    digital_scales(lib::configurator &_config);
    // Destruktor czujnika wirtualnego.
    ~digital_scales(void);
    // Konfiguracja czujnika.
    void configure_sensor (void);
    // Inicjacja odczytu.
    void initiate_reading (void);
    // Odeslanie odczytu.
    void get_reading (void);
    // Watek odczytu pojedynczego linialu
    void worker_thread(int number);
}; // end: class vsp_ds_sensor

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
