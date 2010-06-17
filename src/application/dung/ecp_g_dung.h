// -------------------------------------------------------------------------
//                            generator/ecp_g_test.h
// Deklaracje struktur danych i metod dla procesow ECP z wykorzystaniem sily
//
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_DUNG_H)
#define _ECP_GEN_DUNG_H

#include "lib/impconst.h"		// lib::frame_tab
#include "lib/com_buf.h"			// lib::trajectory_description

#include "ecp/common/generator/ecp_generator.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej dla zadan yoyka z wodzeniem za nos
class dung : public common::generator::generator {

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


public:
	lib::trajectory_description td;
	int step_no;
	double delta[6];

	// konstruktor
	dung(common::task::task& _ecp_task, int step=0);

	virtual bool first_step ();

	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
