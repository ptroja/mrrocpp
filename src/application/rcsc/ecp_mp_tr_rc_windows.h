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
#include <string>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#include "base/ecp_mp/transmitter.h"				// klasa bazowa transmitter
namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

typedef struct
{
	char response[1024];
	char request[255];
	std::string solver_hostname;
	uint16_t solver_port;
	boost::mutex mtx;
} rc_win_buf_typedef;

// wlasciwe pola obrazu - unie!
typedef struct _to_rc_windows
{
	char rc_state[54 + 1]; // 6faces*9facets+trailing '0'
} to_rc_windows_t;

typedef struct _from_rc_windows_t
{
	char sequence[100];
} from_rc_windows_t;

static const std::string TRANSMITTER_RC_WINDOWS = "TRANSMITTER_RC_WINDOWS";

typedef transmitter <to_rc_windows_t, from_rc_windows_t> rc_windows_transmitter_t;

class rc_windows : public rc_windows_transmitter_t
{
private:
	// TODO: rewrite with boost::thread
	pthread_t worker;

	static int make_socket(const std::string & hostname, uint16_t port);
	static void * do_query(void *);

	static rc_win_buf_typedef *rc_win_buf;

public:
	// Konstruktor
	rc_windows(lib::TRANSMITTER_t _transmitter_name, const char* _section_name, task::task& _ecp_mp_object);
	~rc_windows();

	// odczyt z zawieszaniem lub bez
	virtual bool t_read(bool wait);
	// zapis
	virtual bool t_write(void);
};

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp

#endif
