// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_speaker_effector.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- deklaracja klasy edp_speaker_effector
//
// Data:		17.01.2007
// ------------------------------------------------------------------------


#ifndef __EDP_SPEAKER_H
#define __EDP_SPEAKER_H

#include <sys/asoundlib.h>

#include "base/edp/edp_e_manip.h"
#include "robot/speaker/robot_buffers_speaker.h"

namespace mrrocpp {
namespace edp {
namespace speaker {

class speak_t;

// Klasa reprezentujaca speaker'a.
class effector : public common::effector
{
public:
	typedef lib::speaker::c_buffer c_buffer;
	typedef lib::speaker::r_buffer r_buffer ;

	c_buffer instruction;

	speak_t *mt_tt_obj;

	char text2speak[lib::MAX_TEXT]; // MAC 7
	char prosody[lib::MAX_PROSODY]; // MAC 7
	bool speaking; // MAC7
	bool initialize_incorrect;

	//device
	int card;
	int dev;
	snd_pcm_t *pcm_handle;
	int mSamples;
	int mSampleRate;
	int mSampleChannels;
	int mSampleBits;

	int rtn;
	snd_pcm_channel_info_t pi;
	snd_mixer_t *mixer_handle;
	snd_mixer_group_t group;
	snd_pcm_channel_params_t pp;
	snd_pcm_channel_setup_t setup;

	fd_set rfds, wfds;
	short int *piBuffSpeechOut;
	unsigned uicSamplesNo;

	// Konstruktor.
	effector(lib::configurator &_config);
	int init();
	// Destruktor
	virtual ~effector();

	// Interpretuje otrzymana z ECP instrukcje, przygotowuje odpowiedz dla ECP.
	void interpret_instruction(c_buffer &instruction);

	// Ustalenie formatu odpowiedzi.
	lib::REPLY_TYPE rep_type(const c_buffer & instruction);

	// Glowna petla.
	void main_loop();

	// Tworzenie watkow.
	void create_threads();

	// Wypowiedzenie tresci.
	void get_spoken(bool read_hardware, c_buffer & instruction);

	int speak(const c_buffer & instruction);
};

} // namespace speaker
} // namespace edp
} // namespace mrrocpp


#endif
