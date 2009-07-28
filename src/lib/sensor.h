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
	VSP_CONFIGURE_SENSOR, VSP_INITIATE_READING, VSP_GET_READING, VSP_TERMINATE
};
enum VSP_REPORT
{
	VSP_REPLY_OK,
	VSP_SENSOR_NOT_CONFIGURED,
	VSP_READING_NOT_READY,
	INVALID_VSP_COMMAND
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


/*! \struct sensor_image_t
 * \ Structure used for storing and passing sensors data.
 * \author tkornuta
 */
typedef struct sensor_image_t
{
	// wlasciwe pola obrazu - unie!
	union sensor_union_t
	{
		char begin; // pole uzywane jako adres do wlasciwych eementow unii dla memcpy()
		struct
		{
			short reading;
			char text[10];
		} pattern;
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
			short reading;
		} rotation;
		struct
		{
			double readings[6];
		} ds;
		struct
		{
			double frame[16];
		} camera;
		struct
		{
			char colors[9];
		} cube_face;
		struct
		{
			double joy[3];
			char active_motors;
		} pp;
		struct
		{
			int word_id;
		} mic;

		struct
		{
			double frame_O_T_G[16];
			double frame_E_T_G[16];
			double frame_E_r_G[6];
			double frame_E_r_G__CEIH[6];
			double frame_E_r_G__f[6];
			double fEIH_G[8];
		} vis_sac;

		// rcs - rozwiazanie kostki Rubika
		struct
		{
			RCS_INIT init_mode;
			RCS_READING reading_mode;
			char cube_solution[200];
		} rcs;

		//spots recognition
		struct sp_r_t
		{
			double x[4], y[4], z[4], dz, x_sr, y_sr;
			int pic_count;
		} sp_r;

		// tlemanipulacja - vsp_pawel
		struct
		{
			double x, y, z;
			unsigned int nr;
			struct timespec ts;
		} ball;

		// testowy VSP (ptrojane)
		struct
		{
			struct timespec ts;
		} time;

		//uchyb vsp pwilkows
		struct deviation_t{
			int frame_number;
			int x;
			int y;
		}deviation;


		//Obraz fradii dla rotate_gripper
		struct {
			HD_READING reading_state;
			float angle;
		}hd_angle;

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
			float orientation_x;
			float orientation_y;
			float orientation_z;
		} wiimote;

		/// \brief Communication with EdgeShapeAnalyzer (FraDIA)
		/// \author mnowak
		union visioncoordinates_union_t
		{
			/// \brief Array of 8 structures, contains coordinates of found 8 best objects (in SEARCH mode) from FraDIA
			/// \note if we are used less then 8 structures, first unused structure must have distance == 0
			struct Search
			{
				double rot_z;		///< wokol osi Z, wzgledem osi y, w plaszczyznie obrazu (x0y)
				double rot_dev;	///< miedzy osia Z, a prosta 'kamera-obiekt'
				double dist;		///< approximate distance to object
			} search[8];

			/// Structure for response to mrrocpp "is it that object" (is TEST mode) from FraDIA
			struct Test
			{
				bool found;		///< is object found
			} test;

			struct List
			{
				int count;			 ///< count of known objects in structure
				char object[8][32]; ///< known objects
			} list;
		} visioncoordinates_union;

		// struktura z informacja czy znaleziono szachownice
		struct chessboard_t
		{
			int frame_number;
			bool found;
		}chessboard;

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

/// \brief search or test against choosen object (in FraDIA with EdgeShapeAnalyzer)
/// \author mnowak
enum ESA_MODE
{ 
	EM_UNKNOWN = 0,		///< unknown, not used
	EM_SEARCH = 1,		///< search - we get all ROI's with possible interesting objects
	EM_TEST = 2,		///< test - we test object on screen against choosed object from list
	EM_LIST = 3			///< list - we read list of known objects from FraDIA
};

// BUFORY KOMUNIKACYJNE
struct ECP_VSP_MSG
{
	VSP_COMMAND i_code;
	union
	{
		short parameters;

		// Name of the cvFraDIA task.
		char cvfradia_task_name[80];

		// rcs - rozwiazanie kostki Rubika
		struct
		{
			RCS_CONFIGURE configure_mode;
			char cube_state[54];
		} rcs;

		//dane do spots recognition - zadanie wykonania odczytu
		struct
		{
			short command;
			double plate_pos[3];
		} ps_response;

		/// \struct esa
		/// \brief Structure used for choosing mode for FraDIA with EdgeShapeAnalyzer
		/// \author mnowak
		struct
		{
			ESA_MODE mode;		///< mode for EdgeShapeAnalyzer
			union
			{
				char object[32];///< name of object (must be known)
				int offset;     ///< i.e. for LIST
			};
		} esa;			

		// Tryb HaarDetect
		HD_MODE haar_detect_mode;
	};//: koniec unii
};

struct VSP_ECP_MSG
{
	VSP_REPORT vsp_report;
	SENSOR_IMAGE comm_image;
};

/*****************************************************/
// do komunikacji za pomoca devctl()
typedef union
{
	ECP_VSP_MSG to_vsp; // Filled by client on send
	VSP_ECP_MSG from_vsp; // Filled by server on reply
} DEVCTL_MSG;

// ROZKAZY uzywane w devctl()
// odczyt z czujnika
#define DEVCTL_RD __DIOF(_DCMD_MISC, 1, lib::VSP_ECP_MSG)
// zapis do czujnika
#define DEVCTL_WT __DIOT(_DCMD_MISC, 2, lib::ECP_VSP_MSG)
// zapis i odczyt
#define DEVCTL_RW __DIOTF(_DCMD_MISC, 3, lib::DEVCTL_MSG)

// by Y - CZUJNIKI

enum SENSOR_ENUM
{
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
	SENSOR_TIME,
	/*!
	 * Sensor used for communication with the cvFraDIA.
	 */
	SENSOR_CVFRADIA,
	/*!
	 * Sensor used for communication with the PCBird.
	 */
	SENSOR_PCBIRD,
	SENSOR_WIIMOTE,
};

// Klasa obslugi bledow procesu VSP.
class VSP_main_error
{
public:
	const ERROR_CLASS error_class;
	const uint64_t error_no;
	VSP_main_error(ERROR_CLASS err_cl, uint64_t err_no) :
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
	uint32_t union_size;

	// ponizsze zmienne pozwalaja na odczyty z roznym okresem z czujnikow (mierzonym w krokach generatora)
	// w szczegolnosci mozliwe jest unikniecie odczytu po first stepie (nalezy base_period ustawic na 0)
	short base_period; // by Y okresla co ile krokow generatora ma nastapic odczyt z czujnika
	short current_period; // by Y ilosc krokow pozostajaca do odczytu z czujnika

	int pid; // pid vsp
	std::string node_name; // nazwa wezla na ktorym jest powolane vsp

	// Obraz czujnika.
	SENSOR_IMAGE image;
	// Bufor na odczyty otrzymywane z VSP.
	ECP_VSP_MSG to_vsp;
	// Bufor na odczyty otrzymywane z VSP.
	VSP_ECP_MSG from_vsp;
	// Pole do komunikacji za pomoca DEVCTL.
	DEVCTL_MSG devmsg;

	VSP_REPORT vsp_report_aux; //pomocniczy report dla ECP

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

	// Rozkaz zakonczenia procesu VSP.
	virtual void terminate(void)
	{
	}

	virtual ~sensor()
	{
	}

	// Klasa obslugi bledow czujnikow
	class sensor_error
	{
	public:
		const ERROR_CLASS error_class;
		uint64_t error_no;

		sensor_error(ERROR_CLASS err_cl, uint64_t err_no) :
			error_class(err_cl), error_no(err_no)
		{
		}
	};
};

// Przesylka z VSP do EDP
struct VSP_EDP_message
{
	msg_header_t hdr;
	char vsp_name[20];
	short konfigurowac;
};

// Odpowiedz EDP do VSP
struct EDP_VSP_reply
{
	unsigned long servo_step; // by Y numer kroku servo
	double current_present_XYZ_ZYZ_arm_coordinates[6]; // aktualne wspolrzedne XYZ +
	double force[6];
	short force_reading_status; // informacja o odczycie sil
	// EDP_FORCE_SENSOR_OVERLOAD lub EDP_FORCE_SENSOR_READING_ERROR
	// EDP_FORCE_SENSOR_READING_CORRECT

};

} // namespace lib
} // namespace mrrocpp

#endif /* _SENSOR_H */

