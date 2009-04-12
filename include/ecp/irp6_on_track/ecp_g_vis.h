// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP z wykrozsytsaniem sily
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_FORCE_H)
#define _ECP_GEN_FORCE_H

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

class seven_eye_run_linear_generator : public common::ecp_generator {
protected:

	
public:	
  trajectory_description td;
  int step_no;
  double delta[6];
  double frame1[4][4];
  double pose[6][5];
  double pose_v[6][5];
  double pose_a[6][5];
  double pose_d[6][5];
  double pose_d2[6][5];
  bool the_first;
  
  // konstruktor
	seven_eye_run_linear_generator(common::ecp_task& _ecp_task, int step=0);  

  virtual bool first_step ();
      // generuje pierwszy krok ruchu -
      // pierwszy krok czsto rni sie od pozostaych, 
      // np. do jego generacji nie wykorzystuje sie czujnikw
      // (zadanie realizowane przez klas konkretn)
  virtual bool next_step ();
     // generuje kady nastepny krok ruchu
     // (zadanie realizowane przez klas konkretn)
}; // end: class irp6ot_calibration_generator
// --------------------------------------------------------------------------

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
