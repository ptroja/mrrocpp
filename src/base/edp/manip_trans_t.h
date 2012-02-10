// -------------------------------------------------------------------------
//
// Definicje struktur danych i metod dla procesu EDP
//
// -------------------------------------------------------------------------

#ifndef __MANIP_TRANS_T_H
#define __MANIP_TRANS_T_H

#include "base/edp/trans_t.h"

namespace mrrocpp {
namespace edp {
namespace common {

class motor_driven_effector;

class manip_trans_t : public trans_t<>
{
public:
    manip_trans_t(motor_driven_effector& _master);

private:
    motor_driven_effector &master;

    void operator()();
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
