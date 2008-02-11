#ifndef MP_G_PLAYERPOS_H_
#define MP_G_PLAYERPOS_H_

#include "mp/mp_generator.h"
#include "ecp_mp/ecp_mp_tr_player.h"

class mp_playerpos_generator : public mp_generator
{
	private:
		double gx, gy, ga; // GoTo position
		player_transmitter *player_tr; // underlaying Player transmitter

	public:
		mp_playerpos_generator(mp_task& _mp_task, double _gx = 0, double _gy = 0, double _ga = 0);

		bool first_step(void);
		bool next_step(void);
};

#endif /*MP_G_PLAYERPOS_H_*/
