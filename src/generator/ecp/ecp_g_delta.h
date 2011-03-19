// -------------------------------------------------------------------------
//                            generator/ecp_g_jarosz.h dla QNX6
//
// Modyfikacje:
// 1. metody wirtualne w kla sie bazowej sensor - ok. 160
// 2. bonusy do testowania
//
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_DELTA_H)
#define _ECP_GEN_DELTA_H

#include "base/lib/impconst.h"

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// ########################################################################################################
// ########################################################################################################
// ########################### GENERATORY RUCHU DLA ECP (opracowane by Jarosz) ############################
// ########################################################################################################
// ########################################################################################################

// ####################################################################################################
// KLASA BAZOWA dla generatorow o zadany przyrost polozenia/orientacji
// ####################################################################################################

class delta : public common::generator::generator
{
protected:

  double a_max_motor[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_motor[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_joint[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_joint[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_zyz[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_zyz[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_aa[lib::MAX_SERVOS_NR];			// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_aa[lib::MAX_SERVOS_NR];			// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych

public:
	delta(common::task::task& _ecp_task);

   lib::trajectory_description td;

   virtual bool first_step () = 0;
   virtual bool next_step () = 0;

}; // end : class irp6p_ECP_delta_generator


// --------------------------------------------------------------------------


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
