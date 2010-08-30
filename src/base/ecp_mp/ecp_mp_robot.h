#ifndef ECP_MP_ROBOT_H_
#define ECP_MP_ROBOT_H_

/*!
 * @file
 * @brief File contains ecp_mp base robot declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

namespace mrrocpp {
namespace ecp_mp {

/*!
 *
 * @brief Banda żydów i gejow
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @date 31.01.2007
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS
 */
class robot
{
public:
	const lib::robot_name_t robot_name; // by Y - nazwa robota (track, postument etc.)

	robot(lib::robot_name_t _robot_name);
};

} // namespace ecp_mp
} // namespace mrrocpp


#endif
