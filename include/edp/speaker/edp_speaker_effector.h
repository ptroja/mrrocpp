// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_speaker_effector.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- deklaracja klasy edp_speaker_effector
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------


#ifndef __EDP_SPEAKER_H
#define __EDP_SPEAKER_H

// Klasa edp_effector.
#include <pthread.h>
#include <sys/asoundlib.h> //MAC7 - should be before "edp/common/edp.h"
#include "edp/common/edp.h"
//#include "edp/speaker/sound.h" // MAC7



// Klasa reprezentujaca speaker'a.
class edp_speaker_effector  : public edp_effector
{
protected:

    pthread_t edp_tid;
    pthread_t speak_t_tid;
    STATE next_state;   // stan nastepny, do ktorego przejdzie EDP_MASTER

    static void *speak_thread_start(void* arg);
    void *speak_thread(void* arg);

public:

    master_trans_t_buffer *mt_tt_obj;
    void initialize (void);
    char text2speak[MAX_TEXT]; // MAC 7
    char prosody[MAX_PROSODY]; // MAC 7
    bool speaking; // MAC7
    bool initialize_incorrect;

    //device
    int     card;
    int     dev;
    snd_pcm_t *pcm_handle;
    int     mSamples;
    int     mSampleRate;
    int     mSampleChannels;
    int     mSampleBits;

    int     rtn;
    snd_pcm_channel_info_t pi;
    snd_mixer_t *mixer_handle;
    snd_mixer_group_t group;
    snd_pcm_channel_params_t pp;
    snd_pcm_channel_setup_t setup;

    int n;
    fd_set  rfds, wfds;
    struct timespec b_time;
    struct timespec e_time;
    short int *piBuffSpeechOut;
    unsigned uicSamplesNo;

    // Konstruktor.
    edp_speaker_effector (configurator &_config);
    int init ();
    // Destruktor
    virtual ~edp_speaker_effector ();

    // Interpretuje otrzymana z ECP instrukcje, przygotowuje odpowiedz dla ECP.
    void interpret_instruction (c_buffer *instruction);
    // Ustalenie formatu odpowiedzi.
    REPLY_TYPE rep_type (c_buffer *instruction);

    // Glowna petla.
    void main_loop();
    // Tworzenie watkkow.
    void create_threads ();

    // Wypowiedzenie tresci.
    void get_spoken (bool read_hardware, c_buffer *instruction);
    int speak (c_buffer *instruction);

}
; //: edp_speaker_effector

#endif
