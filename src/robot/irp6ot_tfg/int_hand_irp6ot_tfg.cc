// ------------------------------------------------------------------------
//                            int_hand.cc
//
// Funkcja obslugi przerwania -- odczyt i zapis rejestrow sprzetowych dla robota irp6 on_track
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <cctype>
#include <sys/wait.h>
#include <sys/types.h>


#include "robot/irp6ot_tfg/sg_irp6ot_tfg.h"
// Klasa edp_irp6ot_effector.
#include "robot/irp6ot_tfg/edp_irp6ot_tfg_effector.h"
// Klasa hardware_interface.
#include "robot/irp6ot_tfg/hi_irp6ot_tfg.h"

namespace mrrocpp {
namespace edp {
namespace common {

extern irp6ot_tfg::effector* master; // Bufor polecen i odpowiedzi EDP_MASTER

}

namespace irp6ot_tfg {

// ------------------------------------------------------------------------

// Obsluga przerwania sprzetowego

// UWAGA - zmienna ilosc serwomechanizmow w zaleznosci od tego czy gripper jest dolaczony czy nie

// ------------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp

