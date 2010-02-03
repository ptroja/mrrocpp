// -------------------------------------------------------------------------
// Proces:
// Plik:			ecp_mp_t_rc_windows.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		tkornuta
// Data:		10.11.2004
// -------------------------------------------------------------------------


#ifndef __ECP_MP_TR_RC_WINDOWS_H
#define __ECP_MP_TR_RC_WINDOWS_H

#include <pthread.h>

#include "ecp_mp/transmitter/transmitter.h"				// klasa bazowa transmitter
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

typedef struct {
	char response[1024];
	char request[255];
	const char* solver_hostname;
	uint16_t solver_port;
	boost::mutex mtx;
} rc_win_buf_typedef;

/***************** Klasa czujnikow ********************/
class rc_windows: public transmitter{
  private:									// pola do komunikacji
  	pthread_t worker;

	static int make_socket (const char *hostname, uint16_t port);
	static void * do_query(void *);

  	static rc_win_buf_typedef *rc_win_buf;

  public:
	// Konstruktor
 	rc_windows (TRANSMITTER_ENUM _transmitter_name, const char* _section_name, task::task& _ecp_mp_object);
											// konstruktor czujnika virtualnego
	~rc_windows();						// destruktor czujnika virtualnego

    // odczyt z zawieszaniem lub bez
	virtual bool t_read (bool wait);
	// zapis
	virtual bool t_write (void);
};

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp

#endif
