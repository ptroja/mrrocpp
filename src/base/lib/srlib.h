// -------------------------------------------------------------------------
//                            srlib.h
// Definicje struktur danych i metod do komunikacji z SR
//
// -------------------------------------------------------------------------

#if !defined(__SRLIB_H)
#define __SRLIB_H

#include <ctime>
#include <string>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#if defined(USE_MESSIP_SRR)
#include "messip.h"
#endif

#include "base/lib/typedefs.h"
#include "base/lib/com_buf.h"

namespace mrrocpp {
namespace lib {

#define ERROR_TAB_SIZE   2   // Rozmiar tablicy zawierajacej kody bledow
#define NAME_LENGTH     30   // Dlugosc nazwy
const unsigned int TEXT_LENGTH = 256; // Dlugosc tekstu z wiadomoscia do SR

#define SR_BUFFER_LENGHT 10

/* -------------------------------------------------------------------- */
/* Paczka danych przesylanych do procesu SR                             */
/* -------------------------------------------------------------------- */
typedef struct sr_package
{
#if !defined(USE_MESSIP_SRR)
	msg_header_t hdr;
#endif
	struct timespec ts; // czas generacji wiadomosci
	process_type_t process_type; // rodzaj procesu
	int16_t message_type; // typ wiadomosci: blad lub komunikat
	char process_name[NAME_LENGTH]; // nazwa globalna procesu np: /irp6_on_track/EDP1
	char host_name[NAME_LENGTH]; // nazwa hosta na ktorym uruchamiany jest proces
	char description[TEXT_LENGTH]; // tresc wiadomosci
	//  sr_package();
} /*__attribute__((__packed__))*/sr_package_t; // not packed in case of msg_header_t

/* -------------------------------------------------------------------- */
/* Klasa komunikacji z procesem SR                                      */
/* -------------------------------------------------------------------- */

// int global_name_open(const char *name, int flags);// by Y do oblsugi name_open pomiedzy nodami,
// tymczasowe rozwiazanie porponowane przez ekipe QNX

class sr: public boost::noncopyable
{
private:
	boost::condition_variable cond; //! active command condition
	bool has_command; //! flag indicating active command to execute

	void put_one_msg(const lib::sr_package_t& new_msg); // podniesienie semafora
	void get_one_msg(lib::sr_package_t& new_msg); // podniesienie semafora
	bool buffer_empty(); // czy bufor cykliczny jest pusty

	boost::mutex srMutex; //! one-thread a time access mutex

	void wait_for_new_msg(); // oczekiwanie na semafor

	int send_package(void);
	int send_package_to_sr(const sr_package_t& sr_mess);

	const bool multi_thread;
	boost::thread *thread_id;

	void operator()();

	boost::circular_buffer <lib::sr_package_t> cb;
	boost::mutex mtx;

	virtual void interpret() = 0;

#if !defined(USE_MESSIP_SRR)
	int fd; // by W
#else
	messip_channel_t *ch;
#endif /* !USE_MESSIP_SRR */

protected:
	uint64_t error_tab[ERROR_TAB_SIZE]; // tablica slow 64-bitowych zawierajacych kody bledow
	sr_package_t sr_message; // paczka z wiadomoscia dla SR

public:


	sr(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread);
	virtual ~sr(void);
	int message(error_class_t message_type, uint64_t error_code);
	int message(error_class_t message_type, uint64_t error_code0, uint64_t error_code1);
	int message(error_class_t message_type, uint64_t error_code, const std::string & text);
	int message(const std::string & text);
	int message(error_class_t message_type, const std::string & text);
};

class sr_edp: public sr
{
public:
	sr_edp(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread = false);
protected:
	virtual void interpret(void);
};

class sr_ecp: public sr
{
public:
	sr_ecp(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread = false);
protected:
	virtual void interpret(void);
};

// obsluga komunikatow generowanych przez VSP
class sr_vsp: public sr
{
public:
	sr_vsp(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread = false);
protected:
	virtual void interpret(void);
};

// obsluga komunikatow generowanych przez UI// by Y
class sr_ui: public sr
{
public:
	sr_ui(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread = false);
protected:
	virtual void interpret(void);
};

} // namespace lib
} // namespace mrrocpp

#endif
