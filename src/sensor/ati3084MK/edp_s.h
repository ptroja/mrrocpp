// -------------------------------------------------------------------------
//
//
// Definicje klasy edp_ATI3084_force_sensor
// z interfejsem Macieja Kuleszy
//
// Ostatnia modyfikacja: 2005
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_S_ATI3084MK_H)
#define _EDP_S_ATI3084MK_H

#include "base/edp/edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace common {
class manip_effector;
}
namespace sensor {

#define SPEED B115200
#define PORT "/dev/ser1"

/********** klasa czujnikow po stronie VSP **************/
class ATI3084_force : public force
{

public:
	void connect_to_hardware(void);
	ATI3084_force(common::manip_effector &_master);
	virtual ~ATI3084_force();

	void configure_sensor(void); // konfiguracja czujnika
	void wait_for_event(void); // oczekiwanie na zdarzenie
	void initiate_reading(void); // zadanie odczytu od VSP
	void get_reading(void); // odebranie odczytu od VSP		// zwraca blad

private:
	typedef struct _forceReadings
	{
		int16_t ft[6];
	} forceReadings_t;

	int uart, i, r;
	//int licz=0;
	forceReadings_t ftxyz;

	void solve_transducer_controller_failure(void);

	int open_port(void);
	forceReadings_t getFT();
	void sendBias();
}; // end: class vsp_sensor


} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif
