// -------------------------------------------------------------------------
//                            mp_t_haptic.h
// 
// MP task for two robot haptic device
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_T_HAPTIC_H)
#define __MP_T_HAPTIC_H

#include "mp/mp.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

namespace mrrocpp {
namespace mp {
namespace task {

class haptic : public base  
{
protected:
 
      bool configure_edp_force_sensor(bool configure_track, bool configure_postument);
  
public:
	
	haptic(configurator &_config);

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

};


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
