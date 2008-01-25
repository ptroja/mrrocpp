#include "lib/mis_fun.h"

void set_thread_priority(pthread_t thread,  int32_t  sched_priority_l)
{
	int policy;
	struct sched_param param;
	pthread_getschedparam( thread,  &policy, &param );
	param.sched_priority = sched_priority_l;
	pthread_setschedparam(thread,  policy, &param );
}
