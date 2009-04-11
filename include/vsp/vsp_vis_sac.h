// -------------------------------------------------------------------------
//                            vsp_sensor.h		dla QNX6.2
// 
// Definicje klasy vsp_vis_sensor i vsp_error
// 
// Ostatnia modyfikacja: 25.06.2003
// Autor: tkornuta
// -------------------------------------------------------------------------

#if !defined(_VSP_VIS_SAC_H)
#define _VSP_VIS_SAC_H

#include "vsp/vsp_sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********** klasa czujnikow po stronie VSP **************/
class vsp_vis_sac_sensor: public vsp_sensor {

private:
	short zero;							// polozenie zerowe
	
public:
	short ERROR_CODE;
	
	vsp_vis_sac_sensor(configurator &_config);
	~vsp_vis_sac_sensor(void);

	void configure_sensor (void);	// konfiguracja czujnika
	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad

}; // end: class vsp_vis_sensor

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
