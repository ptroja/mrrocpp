// -------------------------------------------------------------------------
//                             ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_on_track
//
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
//
// Ostatnia modyfikacja:
// autor modyfikacji: tstempko
// -------------------------------------------------------------------------

#if !defined(_ECP_irp6_on_track_NURBS_H)
#define _ECP_irp6_on_track_NURBS_H

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/nurbs_tdes.h"
#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

//#############################################################################

class nurbs : public common::generator::generator 
{
protected:
	int mp_communication_mode_; //by Y - 0 bez TASK TERMINATED, 1 - z TASK TERMINATED
	const nurbs_tdes* ntdes_ptr_;
//	nurbs_tdes ntdes_;
	double* EDP_data_next_ptr_;
	double* EDP_data_current_ptr_; 
   	lib::POSE_SPECIFICATION atype_;
public:
	// konstruktor
	nurbs (common::task::task& _ecp_task, const nurbs_tdes& ntdes, int mp_communication_mode_arg = 1);
	
  virtual bool first_step ();
  virtual bool next_step ();

}; //end: class all_irp6ot_linear_generator

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
