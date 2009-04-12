// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_SPEAK_H)
#define _ECP_GEN_SPEAK_H

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {

enum speak_gen_state {SG_AFTER_SET, SG_FIRST_GET, SG_LAST_GET, SG_FINISH};

// --------------------------------------------------------------------------
// Generator mowienia
class speaking_generator : public common::ecp_generator {

protected:

	
public:	
	int step_no;
	double delta[6];
	speak_gen_state new_sg_state, last_sg_state;
	
	// konstruktor
	speaking_generator(common::ecp_task& _ecp_task, int step=0);  
	
	virtual bool first_step ();
	bool configure(const char* text);
	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

#endif
