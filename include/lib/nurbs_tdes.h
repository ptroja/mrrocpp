#ifndef _nurbs_tdes_h
#define _nurbs_tdes_h

#include "lib/nurbs.h"

//by tstempko
class nurbs_tdes { // Opis trajektorii do interpolacji dowolnego typu
private:
	nurbs_tdes& operator=(const nurbs_tdes&);
	nurbs_tdes(const nurbs_tdes&);
public:
  POSE_SPECIFICATION arm_type;  // chyba lepiej uzyc polimorfizmu niz pola typu
  
  int interpolation_node_no;    // Liczba wezlow przy interpolacji
  int internode_step_no;        // Liczba krokow dla jednego przedzialu
  int value_in_step_no;         // Krok, w ktorym zwracana jest odczytana pozycja
  NurbsLib::NurbsCurveAbstract* ncptr;  

  nurbs_tdes(): interpolation_node_no(0), internode_step_no(0), value_in_step_no(0), ncptr(0) {}
};

#endif

