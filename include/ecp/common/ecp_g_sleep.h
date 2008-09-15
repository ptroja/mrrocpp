/*
 * ecp_g_sleep.h
 *
 *Author: Tomasz Bem
 */

#ifndef ECP_G_SLEEP_H_
#define ECP_G_SLEEP_H_

#include <time.h>
#include "ecp/common/ecp_generator.h"

class ecp_sleep_generator : public ecp_generator
{
	private:
		int seconds;		//seconds to wait
		timespec wait;		//structure for nanosleep funciton
		int remain;			//how much time remaining

	public:
		ecp_sleep_generator(ecp_task& _ecp_task, int=1);		//constructor
		bool first_step();		//first step generation
		bool next_step();			//next step generation
		int init_time(int=1);	//initialize time
};

#endif /* ECP_G_SLEEP_H_ */
