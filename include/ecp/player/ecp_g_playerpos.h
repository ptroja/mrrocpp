#if !defined(_ECP_GEN_PLAYERPOS_H)
#define _ECP_GEN_PLAYERPOS_H

#include "ecp/common/ecp_generator.h"
#include "player/playerclient.h"

class playerpos_generator : public ecp_generator
{
	private:
		PlayerClient *client;
		PositionProxy *device;
		
		playerpos_goal_t playerpos_goal;

	public:
		// konstruktor
		playerpos_generator(ecp_task& _ecp_task);
		~playerpos_generator();

		virtual bool first_step ();

		virtual bool next_step ();
		
		void set_goal(playerpos_goal_t &_playerpos_goal);
};

#endif /* _ECP_GEN_PLAYERPOS_H */
