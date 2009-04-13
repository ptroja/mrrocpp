#include "lib/mis_fun.h"

#include <pthread.h>
#include <stdio.h>

namespace mrrocpp {
namespace lib {

void set_thread_priority(pthread_t thread, int sched_priority_l)
{
	int policy;
	struct sched_param param;
	if (pthread_getschedparam(thread, &policy, &param)) {
		perror("pthread_getschedparam()");
	}
	param.sched_priority = sched_priority_l;
	if (pthread_setschedparam(thread, policy, &param)) {
		perror("pthread_setschedparam()");
	}
}

} // namespace lib
} // namespace mrrocpp
