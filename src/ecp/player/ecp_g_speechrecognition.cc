#include <assert.h>
#include <unistd.h>

#include "ecp/player/ecp_g_speechrecognition.h"

namespace mrrocpp {
namespace ecp {
namespace common {

speechrecognition_generator::speechrecognition_generator(ecp_task& _ecp_task)
	: ecp_generator (_ecp_task)
{
	char * hostname = ecp_t.config.return_string_value("player_hostname");
	assert(hostname);
	client = new PlayerClient(hostname, PLAYER_PORTNUM);
	delete [] hostname;
	
	int device_index = ecp_t.config.return_int_value("device_index");
	device = new SpeechRecognitionProxy(client, device_index, 'r');		
}

speechrecognition_generator::~speechrecognition_generator()
{
	delete device;
	delete client;
}

bool speechrecognition_generator::first_step ( )
{
	if (device->fresh) {
			device->Clear();
			device->fresh = false;
	}
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
		strncpy(ecp_t.ecp_reply.commandRecognized, device->rawText, SPEECH_RECOGNITION_TEXT_LEN);
		return false;
	}

	return true;
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


