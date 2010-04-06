/*
 * generator/ecp_g_smb.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SMB_H_
#define ECP_G_SMB_H_

#include <time.h>
#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

class pin_lock : public common::generator::generator
{
	private:
		double waittime;		//seconds to wait
		timespec sleeptime;		//structure for nanosleep funciton
		timespec acttime;
		timespec prevtime;
		timespec starttime;

	public:
		pin_lock(common::task::task& _ecp_task, double=1);		//constructor
		bool first_step();		//first step generation
		bool next_step();			//next step generation
		void init_time(double=1);	//initialize time
};

} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
