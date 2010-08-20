// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(__ECP_MP_TYPEDEF_H)
#define __ECP_MP_TYPEDEF_H

#include <map>

#include "base/lib/impconst.h"
#include "base/lib/sensor_interface.h"

namespace mrrocpp {
namespace ecp_mp {

namespace sensor {
class sensor_interface;
}

namespace transmitter {
class transmitter_base;
}

/**
 * @brief Container type for storing sensors.
 *
 * @ingroup SENSORS
 */
typedef std::map <lib::sensor::SENSOR_t, ecp_mp::sensor::sensor_interface *> sensors_t;

/**
 * @brief Type for Items from sensor container.
 *
 * @ingroup SENSORS
 */
typedef sensors_t::value_type sensor_item_t;

typedef std::map <lib::TRANSMITTER_t, transmitter::transmitter_base*> transmitters_t;
typedef transmitters_t::value_type transmitter_item_t;

} // namespace mp
} // namespace mrrocpp

#endif
