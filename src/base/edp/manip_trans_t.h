
// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __MANIP_TRANS_T_H
#define __MANIP_TRANS_T_H

#include <boost/utility.hpp>

#include "base/edp/trans_t.h"

namespace mrrocpp {
namespace edp {
namespace common {

class motor_driven_effector;

class manip_trans_t : public trans_t
{
private:
    motor_driven_effector &master;

public:
    void operator()();

    manip_trans_t(motor_driven_effector& _master);
    ~manip_trans_t();
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
