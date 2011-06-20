#if !defined(MP_R_POLYCRANK_H_)
#define MP_R_POLYCRANK_H_

#include "base/mp/mp_robot.h"
#include "robot/polycrank/const_polycrank.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class polycrank : public robot
{
public:
	polycrank(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_POLYCRANK_H_*/

/*
#include "base/mp/mp_robot.h"
#include "robot/conveyor/const_conveyor.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class conveyor : public robot
{
public:
	conveyor(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif
*/
