// -------------------------------------------------------------------------
// Proces:		Wszystkie
// Plik:           sensor.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Definicje klasy sensor dla procesow ECP/MP, VSP, EDP
// Autor:		tkornuta
// Data:		17.01.2007
// -------------------------------------------------------------------------

#if !defined(_SENSOR_H)
#define _SENSOR_H

#include <stdint.h>
#include <time.h>

// XXX
#include "lib/srlib.h"

/*********** stale dla wszystkich czujnikow *************/
// Polecenie dla VSP
enum VSP_COMMAND {VSP_CONFIGURE_SENSOR, VSP_INITIATE_READING, VSP_GET_READING, VSP_TERMINATE};
enum VSP_REPORT {VSP_REPLY_OK, VSP_SENSOR_NOT_CONFIGURED, VSP_READING_NOT_READY, INVALID_VSP_COMMAND};

// Odpowiedz od VSP

#define EDP_FORCE_SENSOR_OVERLOAD 2
#define EDP_FORCE_SENSOR_READING_ERROR 1
#define EDP_FORCE_SENSOR_READING_CORRECT 0


typedef enum { RCS_INIT_SUCCESS, RCS_INIT_FAILURE} RCS_INIT;
typedef enum { RCS_SOLUTION_FOUND, RCS_SOLUTION_NOTFOUND, RCS_SOLUTION_NOTNEEDED, RCS_SOLUTION_NOTPOSSIBLE} RCS_READING;


/************ Struktura obrazow czujnika ***************/
typedef struct{
	// wlasciwe pola obrazu - unie!
	union {
		char begin;		// pole uzywane jako adres do wlasciwych eementow unii dla memcpy()
		struct {
			short reading;
			char text[10];
			} pattern;
		struct {
			double rez[6];   // by Y pomiar sily
			short force_reading_status; // informacja o odczycie sil
			// EDP_FORCE_SENSOR_OVERLOAD lub EDP_FORCE_SENSOR_READING_ERROR
			// EDP_FORCE_SENSOR_READING_CORRECT
			int event_type; // zdarzenie wykryte w VSP
			} force;
		struct {
			short reading;
			} rotation;
		struct {
			double readings[6];
			} ds;
		struct {
			double frame[16];
			} camera;
		struct {
			char colors[9];
			} cube_face;
		struct {
			double joy[3];
			char active_motors;
		} pp;
		struct {
			int word_id;
		} mic;

		struct {
			double frame_O_T_G[16];
			double frame_E_T_G[16];
			double frame_E_r_G[6];
			double frame_E_r_G__CEIH[6];
			double frame_E_r_G__f[6];
		} vis_sac;
		
		// rcs - rozwiazanie kostki Rubika
		struct {
			RCS_INIT init_mode;
			RCS_READING reading_mode;
		 	char cube_solution[200];
		} rcs;

		// tlemanipulacja - vsp_pawel
		struct{
			double x, y, z;
			unsigned int nr;
			struct timespec ts;
		} ball;

		// testowy VSP (ptrojane)
		struct {
			struct timespec ts;
		} time;
		
	}; // koniec unii
} SENSOR_IMAGE;

/*****************************************************/

typedef enum { RCS_BUILD_TABLES, RCS_CUBE_STATE } RCS_CONFIGURE;

// BUFORY KOMUNIKACYJNE
struct ECP_VSP_MSG{
	VSP_COMMAND i_code;
	union {
		short parameters;

		// rcs - rozwiazanie kostki Rubika
		struct
		{
			RCS_CONFIGURE configure_mode;
			char cube_state[54];
		} rcs;

	};//: koniec unii
};

struct VSP_ECP_MSG{
	VSP_REPORT vsp_report;
	SENSOR_IMAGE comm_image;
};

/*****************************************************/
// do komunikacji za pomoca devctl()
typedef union {
           ECP_VSP_MSG to_vsp;			// Filled by client on send
           VSP_ECP_MSG from_vsp;		// Filled by server on reply
} DEVCTL_MSG;

// ROZKAZY uzywane w devctl()
// odczyt z czujnika
#define DEVCTL_RD __DIOF(_DCMD_MISC, 1, VSP_ECP_MSG)
// zapis do czujnika
#define DEVCTL_WT __DIOT(_DCMD_MISC, 2, ECP_VSP_MSG)
// zapis i odczyt
#define DEVCTL_RW __DIOTF(_DCMD_MISC, 3, DEVCTL_MSG)


// by Y - CZUJNIKI 

enum SENSOR_ENUM {
	SENSOR_UNDEFINED,
	SENSOR_FORCE_ON_TRACK,
	SENSOR_FORCE_POSTUMENT,
	SENSOR_CAMERA_SA,
	SENSOR_CAMERA_ON_TRACK,
	SENSOR_CAMERA_POSTUMENT,
	SENSOR_GRIPPER_ON_TRACK,
	SENSOR_GRIPPER_POSTUMENT,
	SENSOR_DIGITAL_SCALE_SENSOR,
	SENSOR_FORCE_SENSOR,
	SENSOR_PP,
	SENSOR_MIC,
	SENSOR_PAWEL,
	
	// rcs - VSP znajdujace rozwiazanie dla kostki Rubika
	SENSOR_RCS_KORF,
	SENSOR_RCS_KOCIEMBA,

	// time, testowy czujnik czasu (ptrojane)
	SENSOR_TIME
};

// Klasa obslugi bledow procesu VSP.
class VSP_main_error
{  
    public:
	uint64_t error_class;
	uint64_t error_no;
	VSP_main_error(){};
	VSP_main_error ( uint64_t err_cl, uint64_t err_no)
                 { error_class = err_cl; error_no = err_no; };
}; // end: VSP_main_error


// Klasa bazowa dla czujnikow (klasa abstrakcyjna)
// Czujniki konkretne wyprowadzane sa z klasy bazowej
class sensor
{
  public:
	// Wielkosc przesylanej unii - dla kazdego obrazu inny.
	uint32_t union_size;

   	// ponizsze zmienne pozwalaja na odczyty z roznym okresem z czujnikow (mierzonym w krokach generatora)
  	// w szczegolnosci mozliwe jest unikniecie odczytu po first stepie (nalezy base_period ustawic na 0)
  	short base_period; // by Y okresla co ile krokow generatora ma nastapic odczyt z czujnika
  	short current_period; // by Y ilosc krokow pozostajaca do odczytu z czujnika
  	
	int pid; // pid vsp
	char* node_name; // nazwa wezla na ktorym jest powolane vsp

	// Obraz czujnika.
	SENSOR_IMAGE image;
	// Bufor na odczyty otrzymywane z VSP.
	ECP_VSP_MSG to_vsp;
	// Bufor na odczyty otrzymywane z VSP.
	VSP_ECP_MSG from_vsp;
	// Pole do komunikacji za pomoca DEVCTL.
	DEVCTL_MSG devmsg;			

	// Odebranie odczytu od VSP.
	virtual void get_reading (void)=0;
	// Konfiguracja czujnika.
	virtual void configure_sensor (void){};
	// Zadanie odczytu od VSP.
	virtual void initiate_reading (void){};
	// Rozkaz zakonczenia procesu VSP.
	virtual void terminate (void){};

	virtual ~sensor(){};

	// Klasa obslugi bledow czujnikow
	class sensor_error
	{
	    public:
	      uint64_t error_class;
	      uint64_t error_no;

	      sensor_error ( uint64_t err_cl, uint64_t err_no)
                      { error_class = err_cl; error_no = err_no;};
	}; // end: sensor_error 
}; // end: sensor

#endif /* _SENSOR_H */
