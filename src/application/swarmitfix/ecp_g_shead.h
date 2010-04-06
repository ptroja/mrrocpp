/*
 * generator/ecp_g_head.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SHEAD_H_
#define ECP_G_SHEAD_H_

#include <time.h>
#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

class head_soldify : public common::generator::generator
{
	private:
		double waittime;		//seconds to wait
		timespec sleeptime;		//structure for nanosleep funciton
		timespec acttime;
		timespec prevtime;
		timespec starttime;

	public:
		head_soldify(common::task::task& _ecp_task, double=1);		//constructor
		bool first_step();		//first step generation
		bool next_step();			//next step generation
		void init_time(double=1);	//initialize time
};


class head_desoldify : public common::generator::generator
{
	private:
		double waittime;		//seconds to wait
		timespec sleeptime;		//structure for nanosleep funciton
		timespec acttime;
		timespec prevtime;
		timespec starttime;

	public:
		head_desoldify(common::task::task& _ecp_task, double=1);		//constructor
		bool first_step();		//first step generation
		bool next_step();			//next step generation
		void init_time(double=1);	//initialize time
};


class head_vacuum_on : public common::generator::generator
{
	private:
		double waittime;		//seconds to wait
		timespec sleeptime;		//structure for nanosleep funciton
		timespec acttime;
		timespec prevtime;
		timespec starttime;

	public:
		head_vacuum_on(common::task::task& _ecp_task, double=1);		//constructor
		bool first_step();		//first step generation
		bool next_step();			//next step generation
		void init_time(double=1);	//initialize time
};

class head_vacuum_off : public common::generator::generator
{
	private:
		double waittime;		//seconds to wait
		timespec sleeptime;		//structure for nanosleep funciton
		timespec acttime;
		timespec prevtime;
		timespec starttime;

	public:
		head_vacuum_off(common::task::task& _ecp_task, double=1);		//constructor
		bool first_step();		//first step generation
		bool next_step();			//next step generation
		void init_time(double=1);	//initialize time
};

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
