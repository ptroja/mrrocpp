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
	
	int position_set_cmd_vel(double vx = 0.0, double vy = 0.0, double va = 0.0, int state = 1);
	int position_set_cmd_pose(double gx = 0.0, double gy = 0.0, double ga = 0.0, int state = 1);
	int say(const char *str);
}; 

#endif