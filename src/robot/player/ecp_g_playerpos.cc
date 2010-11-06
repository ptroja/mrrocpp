#include <cassert>
#include <unistd.h>

#include "base/ecp/player/generator/ecp_g_playerpos.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace generator {

playerpos::playerpos(common::task::task& _ecp_task)
	: generator (_ecp_task)
{
	std::string hostname = ecp_t.config.value<std::string>("player_hostname");
	client = new PlayerClient(hostname.c_str(), PLAYER_PORTNUM);
	
	client->SetDataMode(PLAYER_DATAMODE_PULL_NEW);
	
	int device_index = ecp_t.config.value<int>("device_index");
	device = new PositionProxy(client, device_index, 'a');
	device->SelectPositionMode(1);
	
	test_mode = ecp_t.config.value<int>("test_mode");
}

playerpos::~playerpos()
{
	delete device;
	delete client;
}

void playerpos::set_goal(const lib::playerpos_goal_t &_goal)
{
	this->goal = _goal;
}

bool playerpos::first_step()
{
	//device->ResetOdometry();
	printf("playerpos_generator::first_step() -> {%.2f, %.2f, %.2f}\n",
			goal.getX(), goal.getY(), goal.getT());
	device->GoTo(goal.getX(), goal.getY(), goal.getT());

	do {
		client->Read();
	} while (!(device->fresh));
	device->fresh = false;
	
	return true;
}

bool playerpos::next_step()
{
	static bool goto_accepted = false;
	
#if 0
	// do not block
	if (client->Peek(0)) {
		printf("playerpos_generator::next_step():Read()\n");
		client->Read();
	} else {
		printf("playerpos_generator::next_step():usleep()\n");
		usleep(20000);
	}
#else
	// block
	client->Read();
#endif

	if (device->fresh) {
		//device->Print();
		device->fresh = false;
		if (goto_accepted && !device->speed && !device->turnrate)
			return false;
		goto_accepted = true;
	}

	return true;
}

}
} // namespace player
} // namespace ecp
} // namespace mrrocpp

