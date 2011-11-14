/*!
 * @file
 * @brief File containing the declaration of the ATI6284 Froce/Torque sensor class.
 *
 * @author Konrad Banachowicz
 *
 */

#if !defined(_EDP_S_ATI6284_KB_H)
#define _EDP_S_ATI6284_KB_H

#include <comedilib.h>
#include <Eigen/Core>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

#include "base/edp/edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

typedef Matrix <double, 6, 6> Matrix6d;
typedef Matrix <double, 6, 1> Vector6d;

/*!
 * @file
 * @brief File containing the declaration of the ATI3084 Froce/Torque sensor class.
 *
 * @author Konrad Banachowicz
 *
 */

class ATI6284_force : public force
{
public:

	void connect_to_hardware(void);

	ATI6284_force(common::manip_effector &_master);
	virtual ~ATI6284_force();
	void disconnect_from_hardware(void);
	void configure_particular_sensor(void);
	void wait_for_particular_event(void);
	void get_particular_reading(void);

private:
	const std::string dev_name;
	comedi_t *device; // device descriptor
	lsampl_t adc_data[6]; // raw ADC data
	Vector6d datav; // mensured voltage
	Vector6d bias_data; // sensor bias voltage

	comedi_polynomial_t ADC_calib[6]; // ADC calibration polynomial

	Matrix6d conversion_matrix; // F/T conversion matrix
	Vector6d conversion_scale; // F/T scaling

	void convert_data(const Vector6d &result_raw, const Vector6d &bias_raw, lib::Ft_vector &force) const;
}; // end: class vsp_sensor

} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif //_EDP_S_ATI6284_KB_H

