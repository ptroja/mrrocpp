#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/ecp/player/generator/ecp_g_playerpos.h"
#include "base/ecp/player/task/ecp_t_playerpos.h"
#include "robot/player/ecp_mp_t_player.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace task {

// KONSTRUKTORY
playerpos::playerpos(lib::configurator &_config) :
	task(_config)
{
	ppg = new generator::playerpos(*this);
}

playerpos::~playerpos()
{
	delete ppg;
}

void playerpos::mp_2_ecp_next_state_string_handler(void)
{

	switch ((ecp_mp::task::ECP_PLAYER_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
	{
		case ecp_mp::task::ECP_GEN_PLAYERPOS:
			ppg->set_goal(mp_command.ecp_next_state.playerpos_goal);
			ppg->Move();
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
	return new player::task::playerpos(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


