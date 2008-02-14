#include <math.h>
#include <assert.h>

#include "mp/mp_g_playerpos.h"

mp_playerpos_generator::mp_playerpos_generator(mp_task& _mp_task, double _gx, double _gy, double _ga) :
	mp_generator(_mp_task),
	dist_eps(0.1), ang_eps(DTOR(1.0))
{
	set_target(_gx, _gy, _ga);
}


void mp_playerpos_generator::set_target(double _gx, double _gy, double _ga)
{
	gx = _gx;
	gy = _gy;
	ga = _ga;
}

bool mp_playerpos_generator::first_step() {

	player_tr = (player_transmitter *) transmitter_m[TRANSMITTER_PLAYER];
	assert(player_tr);

	player_tr->position_set_cmd_pose(gx, gy, ga);
	
	return true;
}

bool mp_playerpos_generator::next_step() {
	player_tr->t_read(true);
	
	double dist_diff = hypot(
				player_tr->from_va.player_position.px - this->gx,
				player_tr->from_va.player_position.py - this->gy);
				
	double ang_diff = fabs(player_tr->from_va.player_position.pa - this->ga);
	
	printf("goto = [%f,%f,%f]; this = [%f,%f,%f]\n",
			gx, gy, ga,
			player_tr->from_va.player_position.px,
			player_tr->from_va.player_position.py,
			player_tr->from_va.player_position.pa);
	printf("eps = %f, angle_eps = %f\n", dist_diff, RTOD(ang_diff));
	
	return (!(dist_diff < dist_eps && ang_diff < ang_eps)); // finish if close to target
}