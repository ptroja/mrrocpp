// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
// -------------------------------------------------------------------------

#if !defined(__MP_R_SPEAKER_H)
#define __MP_R_SPEAKER_H

#include "mp/mp.h"
#include "lib/robot_consts/speaker_const.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class speaker : public robot
{
public:
	speaker(task::task &mp_object_l); // Konstruktor

};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif
