#if !defined(_ECP_MP_G_SBENCH_TRANSPARENT_H_)
#define _ECP_MP_G_SBENCH_TRANSPARENT_H_

namespace mrrocpp {
namespace ecp_mp {
namespace sbench {
namespace generator {

/*!
 * Name used during sending power supply control commands between MP and ECP.
 */
const std::string POWER_SUPPLY_COMMAND = "POWER_SUPPLY_COMMAND";

/*!
 * Name used during sending cleaning commands between MP and ECP.
 */
const std::string CLEANING_COMMAND = "CLEANING_COMMAND";

} // namespace generator
} // namespace sbench
} // namespace ecp_mp
} // namespace mrrocpp

#endif
