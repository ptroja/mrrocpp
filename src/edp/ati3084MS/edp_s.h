// -------------------------------------------------------------------------
//
//
// Definicje klasy edp_ATI3084_force_sensor
// z interfejsem Macieja Kuleszy
//
// Ostatnia modyfikacja: 2005
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_S_ATI3084MS_H)
#define _EDP_S_ATI3084MS_H

#include "edp/common/edp_irp6s_postument_track.h"

namespace mrrocpp {
namespace edp {
namespace sensor {


typedef struct {
int16_t ft[6];
} forceReadings;

#define SPEED B115200
#define PORT "/dev/ser1"

/********** klasa czujnikow po stronie VSP **************/
class ATI3084_force : public force{

private:


	int uart, i,r;
	//int licz=0;
	forceReadings ftxyz;


	void solve_transducer_controller_failure(void);


	short do_init(void);

	int open_port(void);
	forceReadings getFT(int fd);
	void sendBias(int fd);

public:

	ATI3084_force(common::irp6s_postument_track_effector &_master);
	virtual ~ATI3084_force();

	void configure_sensor (void);	// konfiguracja czujnika
	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad
	void terminate (void);				// rozkaz zakonczenia procesu VSP

}; // end: class vsp_sensor


} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif
