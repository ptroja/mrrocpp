#include "edp_e_spkm1.h"
#include "const_spkm1.h"

namespace mrrocpp {
namespace edp {
namespace spkm1 {

effector::effector(common::shell &_shell) :
	spkm::effector(_shell, lib::spkm1::ROBOT_NAME)
{
}

} // namespace spkm1


namespace common {

// Create spkm effector.
effector* return_created_efector(common::shell &_shell)
{
	return new spkm1::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
