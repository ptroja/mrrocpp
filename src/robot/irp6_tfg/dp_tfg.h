#if !defined(__IRP6_TFG_DATA_PORT_H)
#define __IRP6_TFG_DATA_PORT_H

/*!
 * @file dp_tfg.h
 * @brief File contains data port communication structures for Irp6 two finger grippers
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_tfg irp6p_tfg
 */

namespace mrrocpp {
namespace lib {

struct tfg_command
{
	double desired_position;
};

}
}

#endif
