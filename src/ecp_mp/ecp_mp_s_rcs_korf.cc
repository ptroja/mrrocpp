// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP)
// Plik:	ecp_mp_s_rcs_korf.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:	metody klasy ecp_mp_rcs_korf dla czujnika znajdujacego rozwiazanie kostki Rubika alg. Korfa
// Autor:	jsalacka
// Data:	25.03.2007
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <devctl.h>

#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ecp_mp/ecp_mp_s_rcs_korf.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
rcs_korf::rcs_korf(lib::SENSOR_ENUM _sensor_name, const char* _section_name, task:: base& _ecp_mp_object)
	: base(_sensor_name, _section_name, _ecp_mp_object) {
	// Ustawienie wielkosci przesylanej unii.
	union_size = sizeof(image.sensor_union.rcs);
	// Wyzerowanie odczytow.
    image.sensor_union.rcs.cube_solution[0] = '\0';
} // end: ecp_mp_rcs_korf

/************************** CONFIGURE SENSOR ******************************/
void rcs_korf::configure_sensor() {
	// Rozkaz konfiguracjii czujnika.
	devmsg.to_vsp.i_code= lib::VSP_CONFIGURE_SENSOR;
	memcpy(&devmsg.to_vsp.rcs, &to_vsp.rcs, union_size);
	// Wyslanie polecenia do procesu VSP.
	if (devctl(sd, DEVCTL_RW, &devmsg, sizeof(lib::DEVCTL_MSG), NULL) == 9)
		throw sensor_error(SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
} // end: configure_sensor

/************************** INITIATE READING *********************************/
void rcs_korf::initiate_reading(){
	devmsg.to_vsp.i_code= lib::VSP_INITIATE_READING;
	memcpy(&devmsg.to_vsp.rcs, &to_vsp.rcs, union_size);
	if (devctl(sd, DEVCTL_RW, &devmsg, sizeof(lib::DEVCTL_MSG), NULL) == 9) {
		image.sensor_union.rcs.init_mode = lib::RCS_INIT_FAILURE;
		throw sensor_error(SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}
	if (devmsg.from_vsp.vsp_report == lib::VSP_REPLY_OK) {
		image.sensor_union.rcs.init_mode = lib::RCS_INIT_SUCCESS;
	} else {
		image.sensor_union.rcs.init_mode = lib::RCS_INIT_FAILURE;
		printf("ECP_MP KR initiate_reading: Reply from VSP not OK!\n");
	}
}

/***************************** GET  READING *********************************/
void rcs_korf::get_reading() {
	if(read(sd, &from_vsp, sizeof(lib::VSP_ECP_MSG)) == -1) {
		image.sensor_union.rcs.cube_solution[0] = '\0';
		image.sensor_union.rcs.reading_mode = lib::RCS_SOLUTION_NOTFOUND;
		sr_ecp_msg.message (SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME);
	}
	// jesli odczyt sie powiodl, przepisanie pol obrazu z bufora komunikacyjnego do image;
	if(from_vsp.vsp_report == lib::VSP_REPLY_OK) {
		// Przepisanie pol obrazu z bufora komunikacyjnego do image.
		memcpy(&image.sensor_union.rcs, &from_vsp.comm_image.sensor_union.rcs, union_size);
	} else {
		image.sensor_union.rcs.cube_solution[0] = '\0';
		image.sensor_union.rcs.reading_mode = lib::RCS_SOLUTION_NOTFOUND;
		printf("ECP_MP KR get_reading: Reply from VSP not OK!\n");
	}
} // end: get_reading

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

