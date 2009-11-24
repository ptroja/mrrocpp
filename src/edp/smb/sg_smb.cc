/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "edp/smb/edp_e_smb.h"
#include "edp/smb/sg_smb.h"


namespace mrrocpp {
namespace edp {

namespace smb {

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer (effector &_master) : common::servo_buffer(_master), master(_master)
{

}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
servo_buffer::~servo_buffer(void)
{}
/*-----------------------------------------------------------------------*/

} // namespace smb


namespace common {

servo_buffer* return_created_servo_buffer (manip_and_conv_effector &_master)
{
	return new smb::servo_buffer ((smb::effector &)(_master));
}


} // namespace common
} // namespace edp
} // namespace mrrocpp





