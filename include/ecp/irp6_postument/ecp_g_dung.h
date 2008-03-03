// -------------------------------------------------------------------------
//                            ecp_g_test.h
// Deklaracje struktur danych i metod dla procesow ECP z wykorzystaniem sily
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_DUNG_H)
#define _ECP_GEN_DUNG_H

#include "common/impconst.h"		// frame_tab
#include "common/com_buf.h"			// trajectory_description

#include "ecp/common/ecp_generator.h"

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej dla zadan yoyka z wodzeniem za nos
class dung_generator : public ecp_generator {

private:
	double oq1;
	double oq2;
	double oq3;
	double oq4;
	double oq5;
	double oq6;

protected:
	long run_counter;
	bool second_step;
	frame_tab previous_frame;
	
public:	
	trajectory_description td;
	int step_no;
	double delta[6];
	
	// konstruktor
	dung_generator(ecp_task& _ecp_task, int step=0);
	
	virtual bool first_step ();

	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

#endif
