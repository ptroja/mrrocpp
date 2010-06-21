#if !defined(_ECP_GEN_PLAYERPOS_H)
#define _ECP_GEN_PLAYERPOS_H

#include "base/ecp/ecp_generator.h"
#include "player/playerclient.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace generator {

class playerpos : public common::generator::generator
{
	private:
		PlayerClient *client;
		PositionProxy *device;
		
		lib::playerpos_goal_t goal;
		
		int test_mode;

	public:
		// konstruktor
		playerpos(common::task::task& _ecp_task);
		~playerpos();

		virtual bool first_step ();

		virtual bool next_step ();
		
		void set_goal(const lib::playerpos_goal_t &_goal);
};

}
} // namespace player
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GEN_PLAYERPOS_H */
