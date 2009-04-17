// -------------------------------------------------------------------------
//                            vsp_sensor.h		dla QNX6.2
// 
// Definicje klasy vsp_vis_sensor i vsp_error
// 
// Ostatnia modyfikacja: 25.06.2003
// Autor: tkornuta
// -------------------------------------------------------------------------

#if !defined(_VSP_MIC_H)
#define _VSP_MIC_H

#include "vsp/vsp_sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********** klasa czujnikow po stronie VSP **************/
class mic: public sensor{

private:
	short zero;							// polozenie zerowe
	
public:
	short ERROR_CODE;
	
	mic(lib::configurator &_config);
	~mic(void);

	void configure_sensor (void);	// konfiguracja czujnika
//	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad
	
}; // end: class vsp_vis_sensor

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
