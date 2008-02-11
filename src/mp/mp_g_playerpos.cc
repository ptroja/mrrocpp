#include <math.h>
#include <assert.h>

#include "mp/mp_g_playerpos.h"

mp_playerpos_generator::mp_playerpos_generator(mp_task& _mp_task, double _gx, double _gy, double _ga) :
	mp_generator(_mp_task), gx(_gx), gy(_gy), ga(_ga)
{
}

bool mp_playerpos_generator::first_step() {

	printf("mp_playerpos_generator.transmitter_m.count() = %d @ %s:%d\n", transmitter_m.size(), __FILE__, __LINE__);	
	player_tr = (player_transmitter *) transmitter_m[TRANSMITTER_PLAYER];
	assert(player_tr);

	player_tr->position_set_cmd_pose(gx, gy, ga);
	
	return true;
}

bool mp_playerpos_generator::next_step() {
	player_tr->t_read(true);
	
	double eps = hypot(
				player_tr->from_va.player_position.px - this->gx,
				player_tr->from_va.player_position.py - this->gy);
	
	return (eps < 0.05); // finish if close to target
}