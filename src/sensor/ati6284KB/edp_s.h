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
#include <Eigen/Core>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

#include "base/edp/edp_force_sensor.h"


namespace mrrocpp {
namespace edp {
namespace sensor {

    const std::string dev_name = "/dev/comedi0";

/********** klasa czujnikow po stronie VSP **************/
class ATI6284_force : public force{

private:
        comedi_t *device;
        lsampl_t adc_data[6];
        double datav[6];
        double bias_data[6];

        comedi_polynomial_t ADC_calib[6];

        Matrix<double, 6, 6> conversion_matrix;
        Matrix<double, 6, 1> conversion_scale;

        void convert_data(double result_raw[6], double bias_raw[6], lib::Ft_vector &force);

public:

	void connect_to_hardware (void);

	ATI6284_force(common::manip_effector &_master);
	virtual ~ATI6284_force();

	void configure_sensor (void);	// konfiguracja czujnika
        void wait_for_event(void);	// oczekiwanie na zdarzenie
        void initiate_reading (void);	// zadanie odczytu od VSP
        void get_reading (void);	// odebranie odczytu od VSP		// zwraca blad

}; // end: class vsp_sensor






} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif
