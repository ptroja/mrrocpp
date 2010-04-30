/*
 **  SHEAD.H
 */

#if !defined(__SHEAD_DATA_PORT_H)
#define __SHEAD_DATA_PORT_H

namespace mrrocpp {
namespace lib {

enum SHEAD_HEAD_SOLIDIFICATION {
	SOLIDIFY, DESOLIDIFY, SHEAD_HEAD_SOLIDIFICATION_NO_ACTION
}; // namespace mrrocpp

enum SHEAD_VACUUM_ACTIVATION {
	VACUUM_ON, VACUUM_OFF, SHEAD_VACUUM_ACTIVATION_NO_ACTION
}; // namespace mrrocpp


#define SHEAD_HEAD_SOLIDIFICATION_DATA_PORT "shead_head_soldification_data_port"
#define SHEAD_VACUUM_ACTIVATION_DATA_PORT "shead_vacuum_activation_data_port"
#define SHEAD_REPLY_DATA_REQUEST_PORT "shead_reply_data_request_port"

struct shead_reply {
	bool head_solfified;
	bool vacuum_on;
	bool soldification_in_progress;
	bool vacumization_in_progress;
};

}
}

#endif
