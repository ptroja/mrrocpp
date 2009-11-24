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

#include "edp/spm/edp_e_spm.h"
#include "edp/spm/sg_spm.h"


namespace mrrocpp {
namespace edp {

namespace spm {

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer (effector &_master) : common::servo_buffer(_master), master(_master)
{

}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
servo_buffer::~servo_buffer(void)
{}
/*-----------------------------------------------------------------------*/

} // namespace spm


namespace common {

servo_buffer* return_created_servo_buffer (manip_and_conv_effector &_master)
{
	return new spm::servo_buffer ((spm::effector &)(_master));
}


} // namespace common
} // namespace edp
} // namespace mrrocpp





