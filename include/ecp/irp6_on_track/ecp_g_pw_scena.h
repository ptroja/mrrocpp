// -------------------------------------------------------------------------
//						   Testowe kolo.
// Deklaracje generatora testowego rysujacego koncowka kolo. Koncowka porusz
// sie zgodnie z ruchem wskazowek zegara.
// -------------------------------------------------------------------------

#if !defined(_ECP_G_PW_SCENA_H)
# define _ECP_G_PW_SCENA_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"
#include "common/impconst.h"
#include "common/com_buf.h"
//fradia
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace common {


//enum SCENE_STATE {NOT_RECOGNIZED,RECOGNIZED}

class ecp_g_pw_scena : public ecp_generator
{
	int step_no;
    double next_position[8];

    bool recognized;
    bool down;

    double xdir;
    double ydir;

public:
    ecp_g_pw_scena (ecp_task& _ecp_task);
    double* get_current_pose();
    virtual bool first_step();
    virtual bool next_step();
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif



