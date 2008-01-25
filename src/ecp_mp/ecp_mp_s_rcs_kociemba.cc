// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:	ecp_mp_s_rcs_kociemba.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:	metody klasy ecp_mp_rcs_kociemba dla czujnika znajdujacego rozwiazanie kostki Rubika alg. Kociemby
// Autor:	jsalacka
// Data:	25.03.2007
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <devctl.h>

#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp_mp/ecp_mp_s_rcs_kociemba.h"


/***************************** CONSTRUCTOR ********************************/
ecp_mp_rcs_kociemba::ecp_mp_rcs_kociemba(SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object)
	: ecp_mp_sensor(_sensor_name, _section_name, _ecp_mp_object) {
	// Ustawienie wielkosci przesylanej unii.
	union_size = sizeof(image.rcs);
	// Wyzerowanie odczytow.
	image.rcs.cube_solution[0] = '\0';
}; // end: ecp_mp_digital_scales_sensor

/************************** CONFIGURE SENSOR ******************************/
void ecp_mp_rcs_kociemba::configure_sensor() {
	// Rozkaz konfiguracjii czujnika.
	devmsg.to_vsp.i_code=VSP_CONFIGURE_SENSOR;
	memcpy(&devmsg.to_vsp.rcs, &to_vsp.rcs, union_size);
	//strncpy(devmsg.to_vsp.rcs.cube_state, to_vsp.rcs.cube_state,54);
	// Wyslanie polecenia do procesu VSP.
	if (devctl(sd, DEVCTL_RW, &devmsg, sizeof(DEVCTL_MSG), NULL) == 9)
		throw sensor_error(SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
}; // end: configure_sensor

/************************** INITIATE READING *********************************/
void ecp_mp_rcs_kociemba::initiate_reading(){
	devmsg.to_vsp.i_code=VSP_INITIATE_READING;
	memcpy(&devmsg.to_vsp.rcs, &to_vsp.rcs, union_size);
	if (devctl(sd, DEVCTL_RW, &devmsg, sizeof(DEVCTL_MSG), NULL) == 9) {
		image.rcs.init_mode = RCS_INIT_FAILURE;
		throw sensor_error(SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
	}
	if (devmsg.from_vsp.vsp_report == VSP_REPLY_OK) {
		image.rcs.init_mode = RCS_INIT_SUCCESS;
	} else {
		image.rcs.init_mode = RCS_INIT_FAILURE;
		printf("ECP_MP KR initiate_reading: Reply from VSP not OK!\n");
	}
};

/***************************** GET  READING *********************************/
void ecp_mp_rcs_kociemba::get_reading() {
	if(read(sd, &from_vsp, sizeof(VSP_ECP_MSG)) == -1) {
		image.rcs.cube_solution[0] = '\0';
		image.rcs.reading_mode = RCS_SOLUTION_NOTFOUND;
		sr_ecp_msg.message (SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME);
	}
	// jesli odczyt sie powiodl, przepisanie pol obrazu z bufora komunikacyjnego do image;
	if(from_vsp.vsp_report == VSP_REPLY_OK) {
		// Przepisanie pol obrazu z bufora komunikacyjnego do image.
		memcpy(&image.rcs, &from_vsp.comm_image.rcs, union_size);
	} else {
		image.rcs.cube_solution[0] = '\0';
		image.rcs.reading_mode = RCS_SOLUTION_NOTFOUND;
		printf("ECP_MP KC get_reading: Reply from VSP not OK!\n");
	}
}; // end: get_reading
