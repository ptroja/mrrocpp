#if !defined(_VSP_TIME_SENSOR_H)
#define _VSP_TIME_SENSOR_H

#include <ctime>

#include "base/vsp/vsp_sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********** klasa czujnikow po stronie VSP **************/
class time : public mrrocpp::vsp::common::sensor<struct timespec> {
public:
    // Konstruktor czujnika wirtualnego.
    time (lib::configurator &_config);

    // Konfiguracja czujnika.
    void configure_sensor (void);
    // Inicjacja odczytu.
    void initiate_reading (void);
    // Odeslanie odczytu.
    void get_reading (void);
};

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif /* _VSP_TIME_SENSOR_H */
