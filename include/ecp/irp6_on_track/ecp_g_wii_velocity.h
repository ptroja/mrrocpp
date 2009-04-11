#if !defined_ecp_wii_velocity_generator_H
# define _ecp_wii_velocity_generator_H

#include <string.h>
#include <math.h>
#include <unistd.h>

#include <semaphore.h>
#include <time.h>

#include "ecp/common/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {

#define C_0 0.1
#define C_1 0.1
#define C_2 0.1
#define C_3 0.1
#define C_4 0.1
#define C_5 0.1

class ecp_wii_velocity_generator : public ecp_tff_nose_run_generator
{
	public:
    ecp_wii_velocity_generator (ecp_task& _ecp_task);
    virtual bool first_step();
    virtual bool next_step();
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
