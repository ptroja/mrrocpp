// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_speaker_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot speaker
//				- definicja metod klasy edp_speaker_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------

#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <csignal>
#include <cerrno>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <cerrno>
#include <pthread.h>
#include <process.h>
#include <sys/netmgr.h>

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mis_fun.h"

#include "robot/speaker/sound.h" // MAC7
//#include "robot/speaker/stdafx.h" // MAC& [ARTUR]
#include "robot/speaker/tts.h" // MAC& [ARTUR]
// Klasa edp_speaker_effector.
#include "robot/speaker/edp_speaker_effector.h"
#include "robot/speaker/speak_t.h"
#include "robot/speaker/const_speaker.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace speaker {

effector::effector(lib::configurator &_config) :
	common::effector(_config, lib::speaker::ROBOT_NAME)
{

	real_reply_type = lib::ACKNOWLEDGE;
	// inicjacja deskryptora pliku by 7&Y
	// servo_fd = name_open(lib::EDP_ATTACH_POINT, 0);

	speaking = 0;

	/* Ustawienie priorytetu procesu */

	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 2);
}

int effector::init()
{
	// inicjacja buforow
	msg->message("Initialization in progress ...");

	init_buffers_from_files();

	card = -1;
	dev = 0;

	piBuffSpeechOut = (short int*) calloc(MAXOUTDATA, sizeof(short int));

	setvbuf(stdin, NULL, _IONBF, 0);

	if (card == -1) {
		if ((rtn = snd_pcm_open_preferred(&pcm_handle, &card, &dev, SND_PCM_OPEN_PLAYBACK)) < 0) {
			printf("error: open preffered failed\n");
			return -1;
		}
	} else {
		if ((rtn = snd_pcm_open(&pcm_handle, card, dev, SND_PCM_OPEN_PLAYBACK)) < 0) {
			printf("error: open failed\n");
			return -1;
		}
	}

	mSampleRate = 44000; //16000;
	mSampleChannels = 1;
	mSampleBits = 16;

	// printf ("SampleRate = %d, Channels = %d, SampleBits = %d\n", mSampleRate, mSampleChannels, mSampleBits);
	// disabling mmap is not actually required in this example but it is included to
	// demonstrate how it is used when it is required.
	/*
	 if ((rtn = snd_pcm_plugin_set_disable (pcm_handle, PLUGIN_DISABLE_MMAP)) < 0)
	 {
	 fprintf (stderr, "snd_pcm_plugin_set_disable failed: %s\n", snd_strerror (rtn));
	 return -1;
	 }
	 */

	memset(&pi, 0, sizeof(pi));
	pi.channel = SND_PCM_CHANNEL_PLAYBACK;
	if ((rtn = snd_pcm_plugin_info(pcm_handle, &pi)) < 0) {
		fprintf(stderr, "snd_pcm_plugin_info failed: %s\n", snd_strerror(rtn));
		return -1;
	}

	memset(&pp, 0, sizeof(pp));

	pp.mode = SND_PCM_MODE_BLOCK;
	pp.channel = SND_PCM_CHANNEL_PLAYBACK;
	pp.start_mode = SND_PCM_START_FULL;
	pp.stop_mode = SND_PCM_STOP_STOP;

	pp.buf.block.frag_size = pi.max_fragment_size;
	pp.buf.block.frags_max = 1;
	pp.buf.block.frags_min = 1;

	pp.format.interleave = 1;
	pp.format.rate = mSampleRate;
	pp.format.voices = mSampleChannels;

	if (mSampleBits == 8) {
		pp.format.format = SND_PCM_SFMT_U8;
	} else {
		pp.format.format = SND_PCM_SFMT_S16_LE;
	}

	if ((rtn = snd_pcm_plugin_params(pcm_handle, &pp)) < 0) {
		fprintf(stderr, "snd_pcm_plugin_params failed: %s\n", snd_strerror(rtn));
		return -1;
	}

	if ((rtn = snd_pcm_plugin_prepare(pcm_handle, SND_PCM_CHANNEL_PLAYBACK)) < 0) {
		fprintf(stderr, "snd_pcm_plugin_prepare failed: %s\n", snd_strerror(rtn));
	}

	memset(&setup, 0, sizeof(setup));
	memset(&group, 0, sizeof(group));
	setup.channel = SND_PCM_CHANNEL_PLAYBACK;
	setup.mixer_gid = &group.gid;
	if ((rtn = snd_pcm_plugin_setup(pcm_handle, &setup)) < 0) {
		fprintf(stderr, "snd_pcm_plugin_setup failed: %s\n", snd_strerror(rtn));
		return -1;
	}

	if (group.gid.name[0] == 0) {
		printf("Mixer Pcm Group [%s] Not Set \n", group.gid.name);
		return (-1);
	}

	if ((rtn = snd_mixer_open(&mixer_handle, card, setup.mixer_device)) < 0) {
		fprintf(stderr, "snd_mixer_open failed: %s\n", snd_strerror(rtn));
		return -1;
	}

	FD_ZERO (&rfds);
	FD_ZERO (&wfds);

	FD_SET (STDIN_FILENO, &rfds);
	FD_SET (snd_mixer_file_descriptor (mixer_handle), &rfds);
	FD_SET (snd_pcm_file_descriptor (pcm_handle, SND_PCM_CHANNEL_PLAYBACK), &wfds);

	msg->message("Initialization successful");

	return 0;
}
;

effector::~effector()
{
	;
	free(piBuffSpeechOut);
	rtn = snd_mixer_close(mixer_handle);
	rtn = snd_pcm_close(pcm_handle);
}

void effector::create_threads()
{
	mt_tt_obj = new speak_t(*this);
}

/*--------------------------------------------------------------------------*/
void effector::interpret_instruction(lib::c_buffer &instruction)
{
	// interpretuje otrzyman z ECP instrukcj;
	// wypenaia struktury danych TRANSFORMATORa;
	// przygotowuje odpowied dla ECP
	// 	printf("interpret instruction poczatek\n");
	// wstpne przygotowanie bufora odpowiedzi
	rep_type(instruction); // okreslenie typu odpowiedzi
	reply.error_no.error0 = OK;
	reply.error_no.error1 = OK;

	// Wykonanie instrukcji
	switch (instruction.instruction_type)
	{
		case lib::SET:
			reply.arm.text_def.speaking = speaking;
			if (!speaking) {
				mt_tt_obj->master_to_trans_t_order(common::MT_MOVE_ARM, 0, instruction);
			}
			break;
		case lib::GET:
			reply.arm.text_def.speaking = speaking;
			// mt_tt_obj->master_to_trans_t_order(MT_GET_ARM_POSITION, true);
			break;
		case lib::SET_GET:
			reply.arm.text_def.speaking = speaking;
			mt_tt_obj->master_to_trans_t_order(common::MT_MOVE_ARM, 0, instruction);
			// mt_tt_obj->master_to_trans_t_order(MT_GET_ARM_POSITION, true);
			break;
		default: // blad
			// ustawi numer bledu
			throw NonFatal_error_2(INVALID_INSTRUCTION_TYPE);
	}

	// printf("interpret instruction koniec\n");

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
lib::REPLY_TYPE effector::rep_type(lib::c_buffer & instruction)
{
	// ustalenie formatu odpowiedzi
	reply.reply_type = lib::ACKNOWLEDGE;

	return reply.reply_type;
}
/*--------------------------------------------------------------------------*/

void effector::get_spoken(bool read_hardware, lib::c_buffer *instruction)
{ // MAC7
	return;
}

int effector::speak(lib::c_buffer *instruction)
{ // add by MAC7

	strcpy(text2speak, (*instruction).arm.text_def.text);
	strcpy(prosody, (*instruction).arm.text_def.prosody);

	speaking = 1;

	if (!initialize_incorrect) {
		lib::set_thread_priority(pthread_self(), 2);
		uicSamplesNo = SayIt(text2speak, prosody, piBuffSpeechOut);
		//clock_gettime( CLOCK_REALTIME , &e_time);
		lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 10);
		mSamples = uicSamplesNo * 2; // 42830; // MAC7 sprawdzic, czy dla roznych textow nie bedzie sie roznic // FindTag (file2, "data"); // 441000;

		if (snd_pcm_plugin_prepare(pcm_handle, SND_PCM_CHANNEL_PLAYBACK) < 0) {
			fprintf(stderr, "underrun: playback channel prepare error\n");
			return (-1);
		}

		if (FD_ISSET (snd_pcm_file_descriptor (pcm_handle, SND_PCM_CHANNEL_PLAYBACK), &wfds)) {
			snd_pcm_plugin_write(pcm_handle, piBuffSpeechOut, mSamples);
		}
		n = snd_pcm_plugin_flush(pcm_handle, SND_PCM_CHANNEL_PLAYBACK);
	}
	speaking = 0;

	return 1;
}

void effector::main_loop(void)
{
	common::STATE next_state = common::GET_INSTRUCTION; // MAC7 glosnikow nie trzeba synchronizowac ; )

	/* Nieskoczona petla wykonujca przejscia w grafie automatu (procesu EDP_MASTER) */
	for (;;) {

		try { // w tym bloku beda wylapywane wyjatki (bledy)
			switch (next_state)
			{
				case common::GET_INSTRUCTION:
					switch (receive_instruction())
					{
						case lib::SET:
							// printf("jestesmy w set\n"); // MAC7
						case lib::GET:
							// printf("jestesmy w get\n");// MAC7
						case lib::SET_GET:
							// printf("jestesmy w set_get\n");
							// potwierdzenie przyjecia polecenia (dla ECP)
							reply.reply_type = lib::ACKNOWLEDGE;
							reply_to_instruction();
							break;
						case lib::SYNCHRO: // blad: robot jest juz zsynchronizowany
							// okreslenie numeru bledu
							throw NonFatal_error_1(ALREADY_SYNCHRONISED);
						case lib::QUERY: // blad: nie ma o co pytac - zadne polecenie uprzednio nie zostalo wydane
							// okreslenie numeru bledu
							throw NonFatal_error_1(QUERY_NOT_EXPECTED);
						default: // blad: nieznana instrukcja
							// okreslenie numeru bledu
							throw NonFatal_error_1(UNKNOWN_INSTRUCTION);
					}
					if (reply.reply_type == lib::ERROR)
						printf("ERROR GET_INSTRUCTION 2 aaa\n");
					next_state = common::EXECUTE_INSTRUCTION;
					break;
				case common::EXECUTE_INSTRUCTION:
					// printf("jestesmy w execute instruction\n"); // MAC7
					// wykonanie instrukcji - wszelkie bledy powoduja zgloszenie wyjtku NonFatal_error_2 lub Fatal_error
					interpret_instruction(instruction);
					//      printf("w execute po interpret\n");
					next_state = common::WAIT;
					break;
				case common::WAIT:
					//  	printf("jestesmy w wait\n");

					if (receive_instruction() == lib::QUERY) { // instrukcja wlasciwa =>
						// zle jej wykonanie, czyli wyslij odpowiedz
						reply_to_instruction();
					} else { // blad: powinna byla nadejsc instrukcja QUERY
						throw NonFatal_error_3(QUERY_EXPECTED);
					}
					next_state = common::GET_INSTRUCTION;
					break;
				default:
					break;
			}
		} // end: try

		catch (NonFatal_error_1 & nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// S to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			establish_error(nfe.error, OK);
			// printf("ERROR w EDP 1\n");
			// informacja dla ECP o bledzie
			reply_to_instruction();
			msg->message(lib::NON_FATAL_ERROR, nfe.error, 0);
			// powrot do stanu: GET_INSTRUCTION
			next_state = common::GET_INSTRUCTION;
		}

		catch (NonFatal_error_2 & nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// S to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow

			// printf("ERROR w EDP 2\n");
			establish_error(nfe.error, OK);
			msg->message(lib::NON_FATAL_ERROR, nfe.error, 0);
			// powrot do stanu: WAIT
			next_state = common::WAIT;
		}

		catch (NonFatal_error_3 & nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// S to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			// zapamietanie poprzedniej odpowiedzi
			// Oczekiwano na QUERY a otrzymano co innego, wiec sygnalizacja bledu i
			// dalsze oczekiwanie na QUERY
			lib::REPLY_TYPE rep_type = reply.reply_type;
			uint64_t err_no_0 = reply.error_no.error0;
			uint64_t err_no_1 = reply.error_no.error1;

			establish_error(nfe.error, OK);
			// informacja dla ECP o bledzie
			reply_to_instruction();
			// przywrocenie poprzedniej odpowiedzi
			reply.reply_type = rep_type;
			establish_error(err_no_0, err_no_1);
			// printf("ERROR w EDP 3\n");
			msg->message(lib::NON_FATAL_ERROR, nfe.error, 0);
			// msg->message(lib::NON_FATAL_ERROR, err_no_0, err_no_1); // by Y - oryginalnie
			// powrot do stanu: GET_INSTRUCTION
			next_state = common::GET_INSTRUCTION;
		}

		catch (Fatal_error & fe) {
			// Obsluga bledow fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu fe
			// S to bledy dotyczace sprzetu oraz QNXa (komunikacji)
			establish_error(fe.error0, fe.error1);
			msg->message(lib::FATAL_ERROR, fe.error0, fe.error1);
			// powrot do stanu: WAIT
			next_state = common::WAIT;
		}

		// } // end if important_message_flag // by Y juz zbedne

	} // end: for (;;)
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

namespace mrrocpp {
namespace edp {
namespace common {

effector* return_created_efector(lib::configurator &_config)
{
	return new speaker::effector(_config);
}
;

} // namespace common
} // namespace edp
} // namespace mrrocpp
