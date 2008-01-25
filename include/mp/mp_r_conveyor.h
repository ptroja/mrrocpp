// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
// -------------------------------------------------------------------------

#if !defined(__MP_R_CONVEYOR_H)
#define __MP_R_CONVEYOR_H

#include "mp/mp.h"

// ---------------------------------------------------------------
class mp_conveyor_robot: public mp_robot {

 public:
  mp_conveyor_robot (mp_task* mp_object_l); // Konstruktor

// virtual void execute_motion (void); // Zlecenie wykonania ruchu przez robota
                                      // na poziomie MP jest to polecenie dla ECP
// virtual void terminate_ecp (void); // Zlecenie STOP
 // virtual void start_ecp ( void );      // Zlecenie START

  virtual void create_next_pose_command (void);
    // wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w swych skladowych
	// Ten bufor znajduje sie w robocie

  virtual void get_reply (void);
  virtual void get_input_reply (void);
  virtual void get_arm_reply (void);
  virtual void get_rmodel_reply (void);
    // pobiera z pakietu przeslanego z EDP informacje (aktualnie znajdujace sie
    // w kla sie robot) i wstawia je do odpowiednich swoich skladowych
	// Ten bufor znajduje sie w robocie

}; // end: class mp_conveyor_robot
// --------------------------------------------------------------------------

#endif
