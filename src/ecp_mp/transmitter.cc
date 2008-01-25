// -------------------------------------------------------------------------
//                            ecp_mp_task.cc
//            Effector Control Process (ECP) i MP - methods
// 
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp_mp/ecp_mp_task.h"

transmitter::transmitter(TRANSMITTER_ENUM _transmitter_name, char* _section_name, ecp_mp_task& _ecp_mp_object)
	: transmitter_name(_transmitter_name), sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg)
{
}// end: transmitter

