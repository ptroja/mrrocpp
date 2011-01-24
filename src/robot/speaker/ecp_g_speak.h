// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_SPEAK_H)
#define _ECP_GEN_SPEAK_H

#include "base/ecp/ecp_generator.h"
#include "robot/speaker/ecp_r_speaker.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace generator {

// --------------------------------------------------------------------------
// Generator mowienia

typedef ecp::speaker::robot robot_t;

typedef common::generator::_generator<robot_t> base_generator_t;

class speaking : public base_generator_t {
private:
	enum speak_gen_state {SG_AFTER_SET, SG_FIRST_GET, SG_LAST_GET, SG_FINISH};

	speak_gen_state new_sg_state, last_sg_state;
	
public:
	// konstruktor
	speaking(base_task_t & _ecp_task);
	
	bool configure(const char* text);

	virtual bool first_step ();

	virtual bool next_step ();
};
// --------------------------------------------------------------------------

}
} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

#endif
