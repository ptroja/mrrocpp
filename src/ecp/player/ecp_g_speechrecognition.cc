#include <assert.h>
#include <unistd.h>

#include "ecp/player/ecp_g_speechrecognition.h"

speechrecognition_generator::speechrecognition_generator(ecp_task& _ecp_task)
	: ecp_generator (_ecp_task, true)
{
	hostname = ecp_t.config.return_string_value("player_hostname");
	assert(hostname);

	device_index = ecp_t.config.return_int_value("device_index");

	client = new PlayerClient(hostname, PLAYER_PORTNUM);
	device = new SpeechRecognitionProxy(client, device_index, 'r');		
}

speechrecognition_generator::~speechrecognition_generator()
{
	delete [] hostname;
}

bool speechrecognition_generator::first_step ( )
{
	return true;
}

bool speechrecognition_generator::next_step ( )
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
		device->Clear();
		device->fresh = false;
		return false;
	}

	return true;
}
