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
	device = new PositionProxy(client, device_index, 'r');
}

playerpos_generator::~playerpos_generator()
{
	delete device;
	delete client;
}

void playerpos_generator::set_goal(playerpos_goal_t &_playerpos_goal)
{
	this->playerpos_goal = _playerpos_goal;
}

bool playerpos_generator::first_step ( )
{
	return true;
}

bool playerpos_generator::next_step ( )
{
#if 1
	// do not block
	if(client->Peek(0)) {
		client->Read();
	}
	usleep(20000);
#else
	// block
	client->Read();
#endif
	
	if (device->fresh) {
		device->fresh = false;
		return false;
	}

	return true;
}
