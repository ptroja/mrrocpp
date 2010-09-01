// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota irp6 mechatronika
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_SARKOFAG_H
#define __HI_SARKOFAG_H

#include <csignal>
#include <ctime>

#include "robot/hi_moxa/hi_moxa.h"

namespace mrrocpp {
namespace edp {
namespace sarkofag {

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------


class hardware_interface : public hi_moxa::HI_moxa
{
public:
	hardware_interface(common::motor_driven_effector &_master); // Konstruktor

}; // koniec: class hardware_interface


} // namespace sarkofag
} // namespace edp
} // namespace mrrocpp

#endif // __HI_SARKOFAG_H
