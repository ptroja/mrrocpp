#if !defined(__IRP6_TFG_DATA_PORT_H)
#define __IRP6_TFG_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for Irp6 two finger grippers
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6_tfg
 */

namespace mrrocpp {
namespace lib {
namespace irp6_tfg {

/*!
 * @brief Two finger gripper common mp to ecp command
 * @ingroup irp6_tfg
 */
struct mp_to_ecp_parameters
{
	double desired_position;
};

}
}
}

#endif
