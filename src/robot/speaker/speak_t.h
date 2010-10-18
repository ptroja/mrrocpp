
// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __SPEAK_T_H
#define __SPEAK_T_H

#include "base/edp/trans_t.h"

namespace mrrocpp {
namespace edp {
namespace speaker {

/**************************** trans_t *****************************/

class effector;

class speak_t : public common::trans_t
{
private:
	effector &master;

public:
    void operator()();

    speak_t(effector& _master);
};
/**************************** trans_t *****************************/

} // namespace speaker
} // namespace edp
} // namespace mrrocpp

#endif
