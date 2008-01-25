// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// 
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
// 
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_T_RCSC_IRP6P_H)
#define _ECP_T_RCSC_IRP6P_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp/common/ecp_t_rcsc.h"

class ecp_task_rcsc_irp6p: public ecp_task  {
protected:
	ecp_generator_t* gt;
	ecp_tff_nose_run_generator* nrg;
	ecp_tff_rubik_grab_generator* rgg;
	ecp_tff_gripper_approach_generator* gag;
	ecp_tff_rubik_face_rotate_generator* rfrg;
	ecp_teach_in_generator* tig;
	ecp_smooth_generator* sg;

public:
	// KONSTRUKTORY
	ecp_task_rcsc_irp6p();
	~ecp_task_rcsc_irp6p();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif
