// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego dla robota irp6 postument
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_IRP6P_TFG_H
#define __HI_LOCAL_IRP6P_TFG_H

#include <csignal>
#include <ctime>

#include "robot/hi_moxa/hi_moxa.h"

namespace mrrocpp {
namespace edp {
namespace irp6p_tfg {

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------


class hardware_interface : public hi_moxa::HI_moxa
{
public:
	hardware_interface(common::motor_driven_effector &_master); // Konstruktor

}; // koniec: class hardware_interface

} // namespace irp6p_tfg
} // namespace edp
} // namespace mrrocpp

#endif // __HI_LOCAL_IRP6P_TFG_H
