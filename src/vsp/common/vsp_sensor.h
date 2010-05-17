// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (lib::VSP)
// Plik:			vsp_sensor.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Deklaracja klasy bazowej vsp_sensor - bazy czujnikow po stronie procesu VSP.
// Autor:		tkornuta
// Data:		09.11.2005
// -------------------------------------------------------------------------

#if !defined(_VSP_SENSOR_H)
#define _VSP_SENSOR_H

#include "lib/sensor.h"
// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/*****************************************************/
// do komunikacji za pomoca devctl()
typedef struct {
	char foo[2048];
} DEVCTL_MSG;

// ROZKAZY uzywane w devctl()
// odczyt z czujnika
#define DEVCTL_RD __DIOF(_DCMD_MISC, 1, mrrocpp::vsp::sensor::DEVCTL_MSG)
// zapis do czujnika
#define DEVCTL_WT __DIOT(_DCMD_MISC, 2, mrrocpp::vsp::sensor::DEVCTL_MSG)
// zapis i odczyt
#define DEVCTL_RW __DIOTF(_DCMD_MISC, 3, mrrocpp::vsp::sensor::DEVCTL_MSG)

// Klasa obslugi bledow procesu VSP.
class VSP_main_error
{
public:
	const lib::error_class_t error_class;
	const uint64_t error_no;
	VSP_main_error(lib::error_class_t err_cl, uint64_t err_no) :
		error_class(err_cl), error_no(err_no)
	{
	}
};

/********** klasa czujnikow po stronie VSP **************/
class sensor_interface : public lib::sensor_interface {
protected:
	// Flaga - czy czujnik jest skonfigurowany.
	bool is_sensor_configured;
	// Flaga - czy jakikolwiek odczyt jest gotowy.
	bool is_reading_ready;

public:
	lib::configurator &config;
	lib::sr_vsp *sr_msg;

	const std::string mrrocpp_network_path;

	virtual void set_vsp_report(lib::VSP_REPORT_t) = 0;

	virtual lib::VSP_COMMAND_t get_command(void) const = 0;

	sensor_interface (lib::configurator &_config);

	// Metoda uzywana przy wspolpracy nieinteraktywnej.
	virtual void wait_for_event(void);

	virtual ~sensor_interface(void);

	virtual int msgread(resmgr_context_t *ctp) = 0;

	virtual int msgwrite(resmgr_context_t *ctp) = 0;
};

template <
	typename VSP_ECP_MSG,
	typename ECP_VSP_MSG = lib::empty_t
>
class sensor : public sensor_interface {
protected:
	struct {
		lib::VSP_REPORT_t vsp_report;
		VSP_ECP_MSG comm_image;
	} from_vsp;

	struct {
		lib::VSP_COMMAND_t i_code;
		ECP_VSP_MSG to_vsp;
	} to_vsp;
public:
	sensor(lib::configurator &_config) : sensor_interface(_config)
	{
	}

	int msgread(resmgr_context_t *ctp)
	{
		return resmgr_msgread(ctp, &to_vsp, sizeof(to_vsp), sizeof(struct _io_write));
	}

	int msgwrite(resmgr_context_t *ctp)
	{
		// Count the start address of reply message content.
		/*
		 struct _io_devctl_reply {
		 uint32_t                  zero;
		 int32_t                   ret_val;
		 int32_t                   nbytes;
		 int32_t                   zero2;
		 // char                      data[nbytes];//
		 =>
		 &data = &_io_devctl_reply + 16bytes = &_io_devctl_reply + 4*int
		 */
		return resmgr_msgwrite(ctp, &from_vsp, sizeof(from_vsp), 0);
	}

	void set_vsp_report(lib::VSP_REPORT_t r)
	{
		from_vsp.vsp_report = r;
	}

	lib::VSP_COMMAND_t get_command(void) const
	{
		return to_vsp.i_code;
	}
};

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
sensor_interface * return_created_sensor (lib::configurator &_config);

#define VSP_CREATE_SENSOR(NAME) \
sensor_interface * return_created_sensor (lib::configurator &_config) \
{ \
	return new NAME(_config); \
}

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
