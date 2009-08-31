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

	// check priority range
	int policy_priority_min = sched_get_priority_min(policy);
	if(policy_priority_min == -1) {
		perror("sched_get_priority_min()");
	}

	int policy_priority_max = sched_get_priority_max(policy);
	if(policy_priority_max == -1) {
		perror("sched_get_priority_max()");
	}

	if ((sched_priority_l < policy_priority_min) ||
		(sched_priority_l > policy_priority_max)) {
		fprintf(stderr,
				"requested thread priority (%d) not in <%d:%d> priority range\n",
				sched_priority_l,
				policy_priority_min, policy_priority_max);
	} else {
		param.sched_priority = sched_priority_l;
		if (pthread_setschedparam(thread, policy, &param)) {
			perror("pthread_setschedparam()");
		}
	}
}

} // namespace lib
} // namespace mrrocpp
