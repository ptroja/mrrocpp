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

#include <string>
#include <stdint.h>

#include "lib/com_buf.h"

namespace mrrocpp {
namespace lib {

/*********** stale dla wszystkich czujnikow *************/
// Polecenie dla VSP
typedef enum _VSP_COMMAND
{
	VSP_CONFIGURE_SENSOR,
	VSP_INITIATE_READING,
	VSP_GET_READING,
	VSP_TERMINATE
} VSP_COMMAND_t;

typedef enum _VSP_REPORT
{
	VSP_REPLY_OK,
	VSP_SENSOR_NOT_CONFIGURED,
	VSP_READING_NOT_READY,
	INVALID_VSP_COMMAND
} VSP_REPORT_t;

typedef struct _object_tracker {//unia do lapania kostki
	bool reached;
	bool tracking;
	int x;
	int y;
	int z;
} object_tracker_t;

typedef struct _cube_face
{
	char colors[9];
} cube_face_t;

typedef struct _vis_sac
{
	double frame_O_T_G[16];
	double frame_E_T_G[16];
	double frame_E_r_G[6];
	double frame_E_r_G__CEIH[6];
	double frame_E_r_G__f[6];
	double fEIH_G[8];
} vis_sac_t;

typedef struct _empty {
	//! This is empty data type
} empty_t;

typedef std::string SENSOR_t;

const SENSOR_t SENSOR_CAMERA_SA = "SENSOR_CAMERA_SA";
const SENSOR_t SENSOR_CAMERA_ON_TRACK = "SENSOR_CAMERA_ON_TRACK";
const SENSOR_t SENSOR_CAMERA_POSTUMENT = "SENSOR_CAMERA_POSTUMENT";
const SENSOR_t SENSOR_CVFRADIA = "SENSOR_CVFRADIA";

// Klasa bazowa dla czujnikow (klasa abstrakcyjna)
// Czujniki konkretne wyprowadzane sa z klasy bazowej
class sensor_interface
{
public:
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

	virtual ~sensor_interface()
	{
	}
};

namespace sensor {
// Klasa obslugi bledow czujnikow
class sensor_error
{
public:
	const lib::error_class_t error_class;
	uint64_t error_no;

	sensor_error(lib::error_class_t err_cl, uint64_t err_no) :
		error_class(err_cl), error_no(err_no)
	{
	}
};
} // namespace sensor

} // namespace lib
} // namespace mrrocpp

#endif /* _SENSOR_H */
