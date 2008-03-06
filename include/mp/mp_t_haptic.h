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

class mp_task_haptic : public mp_task  
{
protected:
	bool break_state;
  
      bool configure_edp_force_sensor(bool configure_track, bool configure_postument);
  
public:
	
	mp_task_haptic(configurator &_config);

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

};

#endif
