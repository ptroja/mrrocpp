// -------------------------------------------------------------------------
//                            srlib.h
// Definicje struktur danych i metod do komunikacji z SR
//
// -------------------------------------------------------------------------

#if !defined(__SRLIB_H)
#define __SRLIB_H

#include <time.h>

#include "messip/messip.h"
#include "common/typedefs.h"
#include "common/com_buf.h"

#define SR_MSG_SERVED 0x11	// kod ustawiany po wyswietleniu komunikatu
				// poniewaz przychodza komunikaty o zdarzeniach
				// zwiazanych z oknem i powodowalyby powtorzenie
				// ostatniego komunikatu

#define ERROR_TAB_SIZE   2   // Rozmiar tablicy zawierajacej kody bledow
#define NAME_LENGTH     30   // Dlugosc nazwy

const unsigned int TEXT_LENGTH = 256; // Dlugosc tekstu z wiadomoscia do SR

/* -------------------------------------------------------------------- */
/* Paczka danych przesylanych do procesu SR                             */
/* -------------------------------------------------------------------- */
typedef struct sr_package {
#if !defined(USE_MESSIP_SRR)
  msg_header_t hdr;
#endif
  struct timespec ts;       // czas generacji wiadomosci
  PROCESS_TYPE process_type;      // rodzaj procesu
  int16_t message_type;      // typ wiadomosci: blad lub komunikat
  char process_name[NAME_LENGTH];  // nazwa globalna procesu np: /irp6_on_track/EDP1
  char host_name[NAME_LENGTH]; // nazwa hosta na ktorym uruchamiany jest proces
  char description[TEXT_LENGTH];  // tresc wiadomosci
} sr_package_t;

/* -------------------------------------------------------------------- */
/* Klasa komunikacji z procesem SR                                      */
/* -------------------------------------------------------------------- */

// int global_name_open(const char *name, int flags);// by Y do oblsugi name_open pomiedzy nodami,
// tymczasowe rozwiazanie porponowane przez ekipe QNX

class sr {
#if !defined(USE_MESSIP_SRR)
  int fd;	// by W
#else
  messip_channel_t *ch;
#endif /* !USE_MESSIP_SRR */
protected:
  uint64_t error_tab[ERROR_TAB_SIZE]; // tablica slow 64-bitowych zawierajacych kody bledow
  sr_package_t sr_message;          // paczka z wiadomoscia dla SR
  int send_package(void);

public :
  sr(PROCESS_TYPE process_type, const char *process_name, const char *sr_name);
  virtual ~sr(void);
  int message(int16_t message_type, uint64_t error_code);
  int message(int16_t message_type, uint64_t error_code0, uint64_t error_code1);
  int message(int16_t message_type, uint64_t error_code, const char *text);
  int message(const char *text);
  int message(int16_t message_type, const char *text);
  virtual void interpret() = 0;
};

class sr_edp: public sr {
public:
  sr_edp(PROCESS_TYPE process_type, const char *process_name, const char *sr_name);
  virtual void interpret(void);
};

class sr_ecp: public sr {
public:
  sr_ecp(PROCESS_TYPE process_type, const char *process_name, const char *sr_name);
  virtual void interpret(void);
};

// obsluga komunikatow generowanych przez VSP
class sr_vsp: public sr {
public:
  sr_vsp(PROCESS_TYPE process_type, const char *process_name, const char *sr_name);
  virtual void interpret(void);
};

// obsluga komunikatow generowanych przez UI// by Y
class sr_ui: public sr {
public:
  sr_ui(PROCESS_TYPE process_type, const char *process_name, const char *sr_name);
  virtual void interpret(void);
};

#endif
