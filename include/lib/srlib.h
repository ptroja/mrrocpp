// -------------------------------------------------------------------------
//                            srlib.h
// Definicje struktur danych i metod do komunikacji z SR
// 
// 
// Ostatnia modyfikacja: 2006
// Autor modyfikacji: yoyek
// -------------------------------------------------------------------------

#if !defined(__SRLIB_H)
#define __SRLIB_H

#include <time.h>

#include "messip/messip.h"
#include "common/typedefs.h"

#define SR_MSG_SERVED 0x11  // kod ustawiany po wyswietleniu komunikatu
                             // poniewaz przychodza komunikaty o zdarzeniach
			                 // zwiazanych z oknem i powodowalyby powtorzenie
			                 // ostatniego komunikatu 

#define ERROR_TAB_SIZE   2   // Rozmiar tablicy zawierajacej kody bledow
#define NAME_LENGTH     30   // Dlugosc nazwy

const unsigned int TEXT_LENGTH = 256; // Dlugosc tekstu z wiadomoscia do SR

/* -------------------------------------------------------------------- */
/* Paczka danych przesylanych do procesu SR                             */
/* -------------------------------------------------------------------- */
typedef struct {
#if !defined(USE_MESSIP_SRR)
  msg_header_t hdr;
#endif
  struct timespec ts;       // czas generacji wiadomosci
  word16 process_type;      // rodzaj procesu; EDP, ECP, MP, VSP, UI
  word16 message_type;      // typ wiadomosci: blad lub komunikat
  char process_name[NAME_LENGTH];  // nazwa globalna procesu np: /irp6_on_track/EDP1
  char description[TEXT_LENGTH];  // tresc wiadomosci
} sr_package;

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
  sr_package sr_message;          // paczka z wiadomoscia dla SR
  int send_package(void);
  
public :
  sr(const word16 process_type, const char *process_name, const char *sr_name);
  virtual ~sr(void);
  int message(word16 message_type, uint64_t error_code);
  int message(word16 message_type, uint64_t error_code0, uint64_t error_code1);
  int message(word16 message_type, uint64_t error_code, const char *text);
  int message(const char *text);
  int message(word16 message_type, const char *text);
  virtual void interpret() = 0;
}; // end: sr

class sr_edp: public sr {
public:
  sr_edp(const word16 process_type, const char *process_name, const char *sr_name);
  virtual void interpret(void);  
}; // end: sr_edp

class sr_ecp: public sr {
public:
  sr_ecp(const word16 process_type, const char *process_name, const char *sr_name);
  virtual void interpret(void);  
}; // end: sr_ecp
  

// obsluga komunikatow generowanych przez VSP
class sr_vsp: public sr {
public:
  sr_vsp(const word16 process_type, const char *process_name, const char *sr_name);
  virtual void interpret(void);
}; // end: sr_vsp

// obsluga komunikatow generowanych przez UI// by Y
class sr_ui: public sr {
public:
  sr_ui(const word16 process_type, const char *process_name, const char *sr_name);
  virtual void interpret(void);
}; // end: sr_ui
  
#endif 
