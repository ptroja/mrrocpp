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
#include <string>

#include "lib/srlib.h"

namespace mrrocpp {
namespace lib {

/*********** stale dla wszystkich czujnikow *************/
// Polecenie dla VSP
enum VSP_COMMAND
{
	VSP_CONFIGURE_SENSOR, VSP_INITIATE_READING, VSP_GET_READING, VSP_TERMINATE, VSP_NONE, VSP_FRADIA_CONFIGURE_TASK
};
enum VSP_REPORT
{
	VSP_REPLY_OK,
	VSP_SENSOR_NOT_CONFIGURED,
	VSP_READING_NOT_READY,
	INVALID_VSP_COMMAND,
	VSP_FRADIA_TASK_LOADED,
	VSP_FRADIA_TASK_CONFIGURED
};

// Odpowiedz od VSP

#define EDP_FORCE_SENSOR_OVERLOAD 2
#define EDP_FORCE_SENSOR_READING_ERROR 1
#define EDP_FORCE_SENSOR_READING_CORRECT 0

typedef enum
{
	RCS_INIT_SUCCESS, RCS_INIT_FAILURE
} RCS_INIT;
typedef enum
{
	RCS_SOLUTION_FOUND,
	RCS_SOLUTION_NOTFOUND,
	RCS_SOLUTION_NOTNEEDED,
	RCS_SOLUTION_NOTPOSSIBLE
} RCS_READING;

/*!
 * \enum HD_READING
 * \brief Types commands get from PW_HaarDetect task.
 */
typedef enum { HD_SOLUTION_NOTFOUND, HD_SOLUTION_FOUND } HD_READING;

/*commands from mrrocpp used in Draughts task*/
typedef enum {
	TRACK_PAWN,
	Z_TRACKER,
	DETECT_BOARD_STATE,
	NONE,
	STORE_BOARD,
	CHECK_MOVE
} DRAUGHTS_MODE;

/*information returned to mrrocpp used in Draughts task*/
typedef enum {
	STATE_CHANGED,
	STATE_UNCHANGED,
	STATE_OK,
	BOARD_DETECTION_ERROR
} BOARD_STATUS;

/** Define size of data buffer for FraDIA <-> MRROC++ communication. Used by CommunicationWrapper (FraDIA) and fradia_sensor (MRROC++) */
#define SENSOR_IMAGE_FRADIA_READING_SIZE 100

/*! \struct sensor_image_t
 * \ Structure used for storing and passing sensors data.
 * \author tkornuta
 */
typedef struct sensor_image_t
{
	// wlasciwe pola obrazu - unie!
	union sensor_union_t
	{
		char begin; // pole uzywane jako adres do wlasciwych elementow unii dla memcpy()
		struct
		{
			double rez[6]; // by Y pomiar sily
			short force_reading_status; // informacja o odczycie sil
			// EDP_FORCE_SENSOR_OVERLOAD lub EDP_FORCE_SENSOR_READING_ERROR
			// EDP_FORCE_SENSOR_READING_CORRECT
			int event_type; // zdarzenie wykryte w VSP
		} force;
		struct
		{
			char colors[9];
		} cube_face;

		struct
		{
			double frame_O_T_G[16];
			double frame_E_T_G[16];
			double frame_E_r_G[6];
			double frame_E_r_G__CEIH[6];
			double frame_E_r_G__f[6];
			double fEIH_G[8];
		} vis_sac;

		// testowy VSP (ptrojane)
		struct
		{
			struct timespec ts;
		} time;

		// uchyb vsp pwilkows
		struct deviation_t{
			int frame_number;
			int x;
			int y;
		} deviation;

		// uchyb w follower
		struct {
			bool tracking;
			int x;
			int y;
		} tracker;

		struct object_tracker_t{//unia do lapania kostki
			bool reached;
			bool tracking;
			int x;
			int y;
			int z;
		} object_tracker;

		/*
		 * Structure for storing pawn coordinates from cvFraDIA
		 * used in Draughts task
		 * \author tbem
		 */
		struct {
			char fields[32];
			BOARD_STATUS status;
		} board;

		//Obraz fradii dla rotate_gripper
		struct {
			HD_READING reading_state;
			float angle;
		} hd_angle;

		// struktura z pozycja i katami pcbirda
		struct
		{
			float x, y, z; // pozycja
			float a, b, g; // katy (a = azimuth, b = elevation, g = roll)
			float distance; // odleglosc
			uint32_t ts_sec, ts_usec; // timestamp
		} pcbird;

		//Structure for storing data retrieved from the Wii-mote server
		struct wiimote_t
		{
			int left;
			int right;
			int up;
			int down;
            int buttonA;
            int buttonB;
            int button1;
            int button2;
            int buttonPlus;
            int buttonMinus;
            int buttonHome;
			float orientation_x;
			float orientation_y;
			float orientation_z;
		} wiimote;

		/*!
		 * \struct fradia_t
		 * Structure for storing data retrieved from cvFraDIA.
		 * For testing purposes.
		 * \author tkornuta
		 */
		struct fradia_t
		{
			int x, y, width, height;
		} fradia;

		// struktura z informacja czy znaleziono szachownice
		struct chessboard_t
		{
			int frame_number;
			float transformation_matrix[12];
			bool found;
			bool calibrated;
		} chessboard;

		char fradia_sensor_reading[SENSOR_IMAGE_FRADIA_READING_SIZE];
	} sensor_union; // koniec unii
} SENSOR_IMAGE;

/*****************************************************/

typedef enum
{
	RCS_BUILD_TABLES, RCS_CUBE_STATE
} RCS_CONFIGURE;


/*!
 * \enum HD_MODE
 * \brief Types commands sent to PW_HaarDetect task.
 */
typedef enum{  WITHOUT_ROTATION, PERFORM_ROTATION } HD_MODE;

// BUFORY KOMUNIKACYJNE
/** Define size of data buffer for FraDIA <-> MRROC++ communication. Used by CommunicationWrapper (FraDIA) and fradia_sensor (MRROC++) */
#define ECP_VSP_MSG_FRADIA_COMMAND_SIZE 160

struct ECP_VSP_MSG
{
	VSP_COMMAND i_code;
	union
	{
		short parameters;

		// Name of the cvFraDIA task.
		char cvfradia_task_name[80];

		// structure used to send to fradia data needed for eih calibration
		struct
		{
			int frame_number;
			float transformation_matrix[12];
		} eihcalibration;

		//structure for controlling fraDIA form mrrocpp
		struct {
			DRAUGHTS_MODE draughts_mode;
			char pawn_nr;
		} draughts_control;

		// rcs - rozwiazanie kostki Rubika
		struct
		{
			RCS_CONFIGURE configure_mode;
			char cube_state[54];
		} rcs;

		// Tryb HaarDetect
		HD_MODE haar_detect_mode;

		struct
		{
		  bool led_change;
		  unsigned int led_status;
		  bool rumble;
		} wii_command;

        char fradia_sensor_command[ECP_VSP_MSG_FRADIA_COMMAND_SIZE];
	};//: koniec unii
};

struct VSP_ECP_MSG
{
	VSP_REPORT vsp_report;
	SENSOR_IMAGE comm_image;
};

// by Y - CZUJNIKI

typedef enum _SENSOR_ENUM
{
	SENSOR_UNDEFINED,
	SENSOR_CAMERA_SA,
	SENSOR_CAMERA_ON_TRACK,
	SENSOR_CAMERA_POSTUMENT,
	// rcs - VSP znajdujace rozwiazanie dla kostki Rubika
	SENSOR_RCS_KORF,
	SENSOR_RCS_KOCIEMBA,

	// time, testowy czujnik czasu (ptrojane)
	SENSOR_TIME,
	/*!
	 * Sensor used for communication with the cvFraDIA.
	 */
	SENSOR_CVFRADIA,
	/*!
	 * Sensor used for communication with the PCBird.
	 */
	SENSOR_PCBIRD,
	SENSOR_WIIMOTE
} SENSOR_t;

// Klasa obslugi bledow procesu VSP.
class VSP_main_error
{
public:
	const error_class_t error_class;
	const uint64_t error_no;
	VSP_main_error(error_class_t err_cl, uint64_t err_no) :
		error_class(err_cl), error_no(err_no)
	{
	}
};

// Klasa bazowa dla czujnikow (klasa abstrakcyjna)
// Czujniki konkretne wyprowadzane sa z klasy bazowej
class sensor
{
public:
	// Wielkosc przesylanej unii - dla kazdego obrazu inny.
	size_t union_size;

	// ponizsze zmienne pozwalaja na odczyty z roznym okresem z czujnikow (mierzonym w krokach generatora)
	// w szczegolnosci mozliwe jest unikniecie odczytu po first stepie (nalezy base_period ustawic na 0)
	short base_period; // by Y okresla co ile krokow generatora ma nastapic odczyt z czujnika
	short current_period; // by Y ilosc krokow pozostajaca do odczytu z czujnika

	std::string node_name; // nazwa wezla na ktorym jest powolane vsp

	// Obraz czujnika.
	SENSOR_IMAGE image;
	// Bufor na odczyty otrzymywane z VSP.
	ECP_VSP_MSG to_vsp;
	// Bufor na odczyty otrzymywane z VSP.
	VSP_ECP_MSG from_vsp;

	// Odebranie odczytu od VSP.
	virtual void get_reading(void)=0;
	// Konfiguracja czujnika.
	virtual void configure_sensor(void)
	{
	}

	// Zadanie odczytu od VSP.
	virtual void initiate_reading(void)
	{
	}

	virtual ~sensor()
	{
	}

	// Klasa obslugi bledow czujnikow
	class sensor_error
	{
	public:
		const error_class_t error_class;
		uint64_t error_no;

		sensor_error(error_class_t err_cl, uint64_t err_no) :
			error_class(err_cl), error_no(err_no)
		{
		}
	};
};

} // namespace lib
} // namespace mrrocpp

#endif /* _SENSOR_H */

