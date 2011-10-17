#include "edp_e_spkm2.h"
#include "const_spkm2.h"

namespace mrrocpp {
namespace edp {
namespace spkm2 {

effector::effector(common::shell &_shell) :
	spkm::effector(_shell, lib::spkm2::ROBOT_NAME)
{
}

} // namespace spkm2


namespace common {

// Create spkm effector.
effector* return_created_efector(common::shell &_shell)
{
	return new spkm2::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
