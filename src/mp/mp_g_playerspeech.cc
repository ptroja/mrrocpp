#include <string.h>
#include <assert.h>

#include "mp/mp_g_playerspeech.h"

mp_playerspeech_generator::mp_playerspeech_generator(mp_task& _mp_task, const char *_str) :
	mp_generator(_mp_task)
{
	set_phrase(_str);
}


void mp_playerspeech_generator::set_phrase(const char *_str)
{
	strncpy(string, _str ? _str : "", PLAYER_SPEECH_MAX_STRING_LEN);
}

bool mp_playerspeech_generator::first_step() {

	player_tr = (player_transmitter *) transmitter_m[TRANSMITTER_PLAYER];
	assert(player_tr);

	if (string)
		player_tr->say(string);
	
	return true;
}

bool mp_playerspeech_generator::next_step() {
	player_tr->t_read(true);
	
	return true;
}
