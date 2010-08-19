// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(__MP_H)
#define __MP_H

#include <map>
#include <cstdio>

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/configurator.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class robot;
}

namespace common {

typedef std::map <lib::robot_name_t, robot::robot*> robots_t;
typedef robots_t::value_type robot_pair_t;

} // namespace common

} // namespace mp
} // namespace mrrocpp

// to fix forward declaration issues
#include "base/mp/mp_generator.h"
#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"
#include "lib/com_buf.h"

#endif
