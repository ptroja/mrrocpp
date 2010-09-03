// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
// -------------------------------------------------------------------------

#if !defined(__MP_R_SPEAKER_H)
#define __MP_R_SPEAKER_H

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_robot.h"
#include "robot/speaker/const_speaker.h"

namespace mrrocpp {
namespace mp {
namespace task {
class task;
} // namespace task
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
