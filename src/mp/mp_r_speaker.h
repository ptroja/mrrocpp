// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
// -------------------------------------------------------------------------

#if !defined(__MP_R_SPEAKER_H)
#define __MP_R_SPEAKER_H

#include "mp/mp.h"
namespace mrrocpp {
namespace mp {
namespace robot {
// ---------------------------------------------------------------
class speaker: public robot {

 public:
  speaker (task::task &mp_object_l); // Konstruktor

// virtual void execute_motion (void); // Zlecenie wykonania ruchu przez robota
                                      // na poziomie MP jest to polecenie dla ECP
// virtual void terminate_ecp (void); // Zlecenie STOP
 // virtual void start_ecp ( void );      // Zlecenie START


}; // end: class mp_conveyor_robot
// --------------------------------------------------------------------------
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif
