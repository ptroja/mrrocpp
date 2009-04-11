#if !defined(_ECP_GEN_PLAYERPOS_H)
#define _ECP_GEN_PLAYERPOS_H

#include "ecp/common/ecp_generator.h"
#include "player/playerclient.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class playerpos_generator : public ecp_generator
{
	private:
		PlayerClient *client;
		PositionProxy *device;
		
		playerpos_goal_t goal;
		
		int test_mode;

	public:
		// konstruktor
		playerpos_generator(ecp_task& _ecp_task);
		~playerpos_generator();

		virtual bool first_step ();

		virtual bool next_step ();
		
		void set_goal(const playerpos_goal_t &_goal);
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GEN_PLAYERPOS_H */
