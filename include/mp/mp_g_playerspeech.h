#ifndef MP_G_PLAYERSPEECH_H_
#define MP_G_PLAYERSPEECH_H_

#include "mp/mp_generator.h"
#include "ecp_mp/ecp_mp_tr_player.h"
#include "player/playercommon.h"

class mp_playerspeech_generator : public mp_generator
{
	private:
	    	char string[PLAYER_SPEECH_MAX_STRING_LEN]; // phrase to say
		player_transmitter *player_tr; // underlaying Player transmitter

	public:
		mp_playerspeech_generator(mp_task& _mp_task, const char *_str = NULL);

		bool first_step(void);
		bool next_step(void);
		
		void set_phrase(const char *_str);
};

#endif /*MP_G_PLAYERSPEECH_H_*/
