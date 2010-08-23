#include <cstdio>
#include <cstring>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/ecp/player/generator/ecp_g_speechrecognition.h"
#include "base/ecp/player/task/ecp_t_speechrecognition.h"
#include "robot/player/ecp_mp_t_player.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace task {

// KONSTRUKTORY
speechrecognition::speechrecognition(lib::configurator &_config) :
	task(_config)
{
	srg = new generator::speechrecognition(*this);
}

speechrecognition::~speechrecognition()
{
	delete srg;
}

void speechrecognition::mp_2_ecp_next_state_string_handler(void)
{

	switch ((ecp_mp::task::ECP_PLAYER_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
	{
		case ecp_mp::task::ECP_GEN_SPEECHRECOGNITION:
			srg->Move();
			break;
		default:
			fprintf(stderr, "invalid ecp_next_state.mp_2_ecp_next_state (%d)\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
			break;
	}

}

}
} // namespace player

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new player::task::speechrecognition(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


