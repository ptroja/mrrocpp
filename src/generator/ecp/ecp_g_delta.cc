// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_postument
//
// Ostatnia modyfikacja: 23.02.2005
// zmiana: funkcja WAIT_FOR_STOP -> chyba dziala.
// autor: tkornuta
// -------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <fstream>
#if defined(__QNXNTO__)
#include <sys/neutrino.h>
#endif /* __QNXNTO__ */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

#include "lib/srlib.h"
#include "generator/ecp/ecp_g_delta.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

delta::delta(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
}


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
