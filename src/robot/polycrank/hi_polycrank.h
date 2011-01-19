// -------------------------------------------------------------------------
//                            hi_polycrank.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota polycrank
//
// Ostatnia modyfikacja: 13.01.2011
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_CONV_H
#define __HI_LOCAL_CONV_H

#include <csignal>
#include <ctime>

#include "robot/hi_moxa/hi_moxa.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------


class hardware_interface : public hi_moxa::HI_moxa
{
public:
	hardware_interface(common::motor_driven_effector &_master); // Konstruktor

}; // koniec: class hardware_interface


} // namespace polycrank
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
