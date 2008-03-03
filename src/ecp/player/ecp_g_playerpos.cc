#include <assert.h>
#include <unistd.h>

#include "ecp/player/ecp_g_playerpos.h"

playerpos_generator::playerpos_generator(ecp_task& _ecp_task)
	: ecp_generator (_ecp_task, true)
{
	char *hostname = ecp_t.config.return_string_value("player_hostname");
	assert(hostname);
	client = new PlayerClient(hostname, PLAYER_PORTNUM);
	delete [] hostname;

	int device_index = ecp_t.config.return_int_value("device_index");
	device = new PositionProxy(client, device_index, 'a');
	
	test_mode = ecp_t.config.return_int_value("test_mode");
}

playerpos_generator::~playerpos_generator()
{
	delete device;
	delete client;
}

void playerpos_generator::set_goal(playerpos_goal_t &_goal)
{
	printf("playerpos_generator::set_goal({%.2f, %.2f, %.2f})\n",
			_goal.x, _goal.y, _goal.t);
	this->goal = _goal;
}

bool playerpos_generator::first_step()
{
	device->SelectPositionMode(1);
	printf("playerpos_generator::first_step() -> {%.2f, %.2f, %.2f}\n",
			goal.x, goal.y, goal.t);
	device->GoTo(goal.x, goal.y, goal.t);
	return true;
}

bool playerpos_generator::next_step()
{
#if 0
	// do not block
	if (client->Peek(0)) {
		client->Read();
	}
	usleep(20000);
#else
	// block
	client->Read();
#endif

	if (device->fresh) {
		device->fresh = false;
		return (device->speed || device->turnrate);
	}

	return true;
}
