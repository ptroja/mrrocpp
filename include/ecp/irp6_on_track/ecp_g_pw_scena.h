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

class ecp_g_pw_scena : public ecp_generator
{
	//Buffer used to communicate between ECP and VSP
//	ECP_VSP_MSG communication_buffer;
//	ecp_mp_cvfradia_sensor * sensor;
	//Object coordinates
//	SENSOR_IMAGE object_descriptor;

    trajectory_description td;
    double next_position[8];

public:
    ecp_g_pw_scena (ecp_task& _ecp_task);
    virtual bool first_step();
    virtual bool next_step();
};

#endif



