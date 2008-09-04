/*
 * ecp_g_sleep.h
 *
 *  Created on: Sep 3, 2008
 *      Author: ghard
 */

#ifndef ECP_G_SLEEP_H_
#define ECP_G_SLEEP_H_

#include <time.h>
#include "ecp/common/ecp_generator.h"

class ecp_sleep_generator : public ecp_generator
{
	int wait_time;
	int current_time;

	int seconds;

  public:
	ecp_sleep_generator(ecp_task& _ecp_task, int=1);
	bool first_step();
	bool next_step();

	int init_time(int=1);
	bool is_now();
};

#endif /* ECP_G_SLEEP_H_ */
