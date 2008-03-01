#include <assert.h>
#include <unistd.h>

#include "ecp/player/ecp_g_speechrecognition.h"

speechrecognition_generator::speechrecognition_generator(ecp_task& _ecp_task)
	: ecp_generator (_ecp_task, true)
{
	hostname = ecp_t.config->return_string_value("player_hostname");
	assert(hostname);

	device_index = ecp_t.config->return_int_value("device_index");

	client = new PlayerClient(hostname, PLAYER_PORTNUM);
	device = new SpeechRecognitionProxy(client, device_index, 'r');		
}

speechrecognition_generator::~speechrecognition_generator()
{
	delete [] hostname;
}

bool speechrecognition_generator::first_step ( )
{
	ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);

	ecp_t.mp_buffer_receive_and_send ();

	switch ( ecp_t.mp_command_type() ) {
		case NEXT_POSE:
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			printf("first_step()::INVALID_MP_COMMAND = %d\n", INVALID_MP_COMMAND);
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	return true;
}

bool speechrecognition_generator::next_step ( )
{
	if (ecp_t.pulse_check()) {
		ecp_t.mp_buffer_receive_and_send ();
		return false;
	} else {
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.mp_buffer_receive_and_send ();
	}

	switch ( ecp_t.mp_command_type() ) {
		case NEXT_POSE:
			//the_robot->create_command ();
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

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
