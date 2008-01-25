// -------------------------------------------------------------------------
// Proces: 	
// Plik:			ecp_mp_t_player.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		tkornuta
// Data:		10.11.2004
// -------------------------------------------------------------------------

#ifndef __ECP_MP_TR_RC_WINDOWS_H
#define __ECP_MP_TR_RC_WINDOWS_H

#include <pthread.h>
#include <semaphore.h>

#include "ecp_mp/transmitter.h"				// klasa bazowa transmitter
#include "player/playerc.h"

/***************** Klasa czujnikow ********************/
class player_transmitter: public transmitter{
  private:						// pola do komunikacji
    pthread_t worker;
    pthread_cond_t cond;
    pthread_mutex_t mtx;
    playerc_client_t *client;   // client proxy
    playerc_device_t *device; // interface proxy
    int if_code;                // interface code
    static void * query_loop (void * arg);
  	
  public:
	// Konstruktor
 	player_transmitter (
            TRANSMITTER_ENUM _transmitter_name, char* _section_name, ecp_mp_task& _ecp_mp_object,
            const char *host, unsigned int port,
            const char *devname, int devindex, int access);
											// konstruktor czujnika virtualnego
	~player_transmitter();						// destruktor czujnika virtualnego

    // odczyt z zawieszaniem lub bez
	virtual bool t_read (bool wait);
	// zapis
	virtual bool t_write (void);
}; 

#endif
