#if !defined(__EPOS_DATA_PORT_H)
#define __EPOS_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for processes cooperating with Epos2 Maxon controllers
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup epos
 */

#include <string>

namespace mrrocpp {
namespace lib {
namespace epos {

/*!
 * @brief SwarmItFix Epos total number of servos
 * @ingroup epos
 */
static const int EPOS_DATA_PORT_SERVOS_NUMBER = 7;

/*!
 * @brief SwarmItFix Epos generator possible profiles enum
 * @ingroup epos
 */
enum EPOS_GEN_PROFILE
{
	TRAPEZOIDAL_VELOCITY, CUBIC_POSITION, OPERATIONAL_SPACE, EPOS_GEN_PROFILE_NO_ACTION
};

/*!
 * @brief SwarmItFix Epos cubic trajectory command data port
 * @ingroup epos
 */
const std::string EPOS_CUBIC_COMMAND_DATA_PORT = "EPOS_CUBIC_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos trapezoidal trajectory command data port
 * @ingroup epos
 */
const std::string EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT = "EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos operational space trajectory command data port
 * @ingroup epos
 */
const std::string EPOS_OPERATIONAL_COMMAND_DATA_PORT = "EPOS_OPERATIONAL_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos motor brake command data port
 * @ingroup epos
 */
const std::string EPOS_BRAKE_COMMAND_DATA_PORT = "EPOS_BRAKE_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos status data request port
 * @ingroup epos
 */
const std::string EPOS_REPLY_DATA_REQUEST_PORT = "EPOS_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Epos controller mp to ecp command
 * @ingroup epos
 */
struct mp_to_ecp_cubic_trapezoidal_parameters
{
	double dm;
	double aa;
	double da;
	double mv;
};

/*!
 * @brief SwarmItFix Epos single controller status
 * @ingroup epos
 */
struct single_controller_epos_reply
{
	double position;
	bool motion_in_progress;
	bool buffer_full;
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Epos cubic trajectory command
 * @ingroup epos
 */
struct epos_cubic_command
{
	double emdm[EPOS_DATA_PORT_SERVOS_NUMBER];
	double aa[EPOS_DATA_PORT_SERVOS_NUMBER];
	double da[EPOS_DATA_PORT_SERVOS_NUMBER];
	double av[EPOS_DATA_PORT_SERVOS_NUMBER];
};

/*!
 * @brief SwarmItFix Epos trapezoidal trajectory command
 * @ingroup epos
 */
struct epos_trapezoidal_command
{
	double em[EPOS_DATA_PORT_SERVOS_NUMBER];
	double emdm[EPOS_DATA_PORT_SERVOS_NUMBER];
	double tt;
};

/*!
 * @brief SwarmItFix Epos operational trajectory command data port
 * @ingroup epos
 */
struct epos_operational_command
{
	double em[EPOS_DATA_PORT_SERVOS_NUMBER];
	double v[EPOS_DATA_PORT_SERVOS_NUMBER];
	double tau;
};

/*!
 * @brief SwarmItFix Epos all controllers status
 * @ingroup epos
 */
struct epos_reply
{
	single_controller_epos_reply epos_controller[EPOS_DATA_PORT_SERVOS_NUMBER];
	bool contact;
};

} // namespace epos
} // namespace lib
} // namespace mrrocpp

#endif
