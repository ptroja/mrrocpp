#ifndef __ECP_MP_TR_RC_WINDOWS_H
#define __ECP_MP_TR_RC_WINDOWS_H

#include <pthread.h>

#include "ecp_mp/transmitter/transmitter.h"				// klasa bazowa transmitter
#include "player/playerc.h"

playerc_joystick_t player_joystick;
playerc_position_t player_position;
playerc_speech_recognition_t player_speech_recognition;

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

static const std::string TRANSMITTER_PLAYER = "TRANSMITTER_PLAYER";

/***************** Klasa czujnikow ********************/
class player: public transmitter {
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
 	player (
            TRANSMITTER_ENUM _transmitter_name, const char* _section_name, task::task& _ecp_mp_object,
            const char *host, unsigned int port,
            const char *devname, int devindex, int access);
											// konstruktor czujnika virtualnego
	~player();						// destruktor czujnika virtualnego

    // odczyt z zawieszaniem lub bez
	virtual bool t_read (bool wait);
	// zapis
	virtual bool t_write (void);
	
	int position_set_cmd_vel(double vx = 0.0, double vy = 0.0, double va = 0.0, int state = 1);
	int position_set_cmd_pose(double gx = 0.0, double gy = 0.0, double ga = 0.0, int state = 1);
	int say(const char *str);
}; 

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp

#endif
