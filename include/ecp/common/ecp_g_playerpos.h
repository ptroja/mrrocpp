#if !defined(_ECP_GEN_PLAYERPOS_H)
#define _ECP_GEN_PLAYERPOS_H

#include "player/playerclient.h"
#include "ecp/common/ecp_generator.h"

class playerpos_generator : public ecp_generator
{
	private:
		PlayerClient *pc;
		PositionProxy *pp;

	public:
		// konstruktor
		playerpos_generator(ecp_task& _ecp_task);

		virtual bool first_step ();

		virtual bool next_step ();

};

#endif /* _ECP_GEN_PLAYERPOS_H */
