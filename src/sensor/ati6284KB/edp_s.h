// -------------------------------------------------------------------------
//
//
// Definicje klasy edp_ATI6284_force_sensor
//
// Ostatnia modyfikacja: styczen 2010
// Autor: labi (Kamil Tarkowski)
// Autor: Yoyek (Tomek Winiarski)
// -------------------------------------------------------------------------


#if !defined(_EDP_S_ATI6284_MS_H)
#define _EDP_S_ATI6284_MS_H

#include <comedilib.h>

#include "base/edp/edp_force_sensor.h"


namespace mrrocpp {
namespace edp {
namespace sensor {

/* macierz konwersji napiecia z czujnika na sile i momenty sil */
double conversion_scale[6] = {
   4.5511972116989,
   4.5511972116989,
   1.41244051397552,
  84.8843245576086,
  84.8843245576086,
  80.9472037525247
};

double conversion_matrix[6][6] = {
  {-0.40709,  -0.27318,   0.34868, -33.58156,  -0.32609,  33.54162},
  { 0.35472,  38.22730,  -0.41173, -19.49156,   0.49550, -19.15271},
  {18.72635,  -0.59676,  19.27843,  -0.56931,  18.69352,  -0.67633},
  {-0.40836,  -0.95908, -33.37957,   1.38537,  32.52522,  -0.51156},
  {37.13715,  -1.02875, -20.00474,  -0.27959, -19.34135,   1.42577},
  {-0.15775, -18.16831,  -0.00133, -18.78961,   0.31895, -18.38586}
};



/********** klasa czujnikow po stronie VSP **************/
class ATI6284_force : public force{

private:
        comedi_t *device;
        lsampl_t adc_data[6];
        double datav[6];
        double bias_data[6];

        comedi_polynomial_t ADC_calib[6];

public:

	void connect_to_hardware (void);

	ATI6284_force(common::manip_effector &_master);
	virtual ~ATI6284_force();

	void configure_sensor (void);	// konfiguracja czujnika
	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad
}; // end: class vsp_sensor


void convert_data(double result_raw[6], double bias_raw[6], double force[6]);



} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif
