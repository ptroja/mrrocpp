// -------------------------------------------------------------------------
//                            ecp_mp_sensor.cc
//            Effector Control Process (ECP) i MP - methods
// 
// Wlasciwy konstruktor czujnika wirtualnego.
// -------------------------------------------------------------------------

#include "ecp_mp/ecp_mp_sensor.h"
#include "ecp_mp/ecp_mp_task.h"

#include "messip/messip.h"

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

ecp_mp_sensor::ecp_mp_sensor(SENSOR_ENUM _sensor_name, const char* _section_name, ecp_mp_task& _ecp_mp_object)
	: sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
	// cout<<"ecp_mp_sensor - konstruktor: "<<_section_name<<endl;

	// Ustawienie domyslnego okresu pracy czujnika.
	base_period=current_period=1;

	node_name = _ecp_mp_object.config->return_string_value("node_name", _section_name);

#if !defined(USE_MESSIP_SRR)
	VSP_NAME = _ecp_mp_object.config->return_attach_point_name(configurator::CONFIG_RESOURCEMAN_GLOBAL, "resourceman_attach_point", _section_name);

 	// cout<<"VSP_NAME = "<<VSP_NAME<<endl;

	// Sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow o tej nazwie.
	if( access(VSP_NAME, R_OK)==0 )
	{
		// by Y - usuniete bo mozna podlaczyc sie do istniejacego czujnika
		// throw sensor_error(SYSTEM_ERROR, DEVICE_ALREADY_EXISTS);
		pid = 0; // tymczasowo
	} else {
	// Stworzenie nowego procesu.
	if ((pid = _ecp_mp_object.config->process_spawn(_section_name)) == -1)
		throw sensor_error(SYSTEM_ERROR, CANNOT_SPAWN_VSP);
		// Proba otworzenie urzadzenia.
	}

	short tmp = 0;
 	// Kilka sekund  (~2) na otworzenie urzadzenia.
	while( (sd = open(VSP_NAME, O_RDWR)) == -1)
	{
// 		cout<<tmp<<endl;
		if((tmp++)<CONNECT_RETRY)
			usleep(1000*CONNECT_DELAY);
		else				
			throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);						
	}// end: while
#else /* USE_MESSIP_SRR */
	VSP_NAME = _ecp_mp_object.config->return_attach_point_name(configurator::CONFIG_SERVER, "resourceman_attach_point", _section_name);

 	// cout<<"VSP_NAME = "<<VSP_NAME<<endl;

	// Sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow o tej nazwie.
	if( (ch = messip_channel_connect(NULL, VSP_NAME, MESSIP_NOTIMEOUT)))
	{
		// by Y - usuniete bo mozna podlaczyc sie do istniejacego czujnika
		// throw sensor_error(SYSTEM_ERROR, DEVICE_ALREADY_EXISTS);
		pid = 0; // tymczasowo
		return;
	}

	// Stworzenie nowego procesu.
	if ((pid = _ecp_mp_object.config->process_spawn(_section_name)) == -1)
		throw sensor_error(SYSTEM_ERROR, CANNOT_SPAWN_VSP);

	short tmp = 0;
 	// Kilka sekund  (~2) na otworzenie urzadzenia.
	while( (ch = messip_channel_connect(NULL, VSP_NAME, MESSIP_NOTIMEOUT)) == NULL)
	{
// 		cout<<tmp<<endl;
		if((tmp++)<CONNECT_RETRY)
			usleep(1000*CONNECT_DELAY);
		else {
			fprintf(stderr, "ecp_mp_sensor: messip_channel_connect(%s) failed\n", VSP_NAME);
			throw sensor_error(SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);						
		}
	}// end: while
#endif /* !USE_MESSIP_SRR */
}// end: ecp_mp_sensor

void ecp_mp_sensor::terminate() {
	to_vsp.i_code=VSP_TERMINATE;
#if !defined(USE_MESSIP_SRR)
	if(write(sd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
		sr_ecp_msg.message (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);   
	else
		close(sd);
#else /* USE_MESSIP_SRR */
	int status;
	if(messip_send(ch, 0, 0, &to_vsp, sizeof(ECP_VSP_MSG),
				&status, &from_vsp, sizeof(VSP_ECP_MSG), MESSIP_NOTIMEOUT) < 0)
		sr_ecp_msg.message (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);   
	else
		messip_channel_disconnect(ch, MESSIP_NOTIMEOUT);
#endif /* !USE_MESSIP_SRR */
} // end: terminate()

void ecp_mp_sensor::initiate_reading() {
	to_vsp.i_code=VSP_INITIATE_READING;
#if !defined(USE_MESSIP_SRR)
	if(write(sd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
#else /* USE_MESSIP_SRR */
	int status;
	if(messip_send(ch, 0, 0, &to_vsp, sizeof(ECP_VSP_MSG),
				&status, &from_vsp, sizeof(VSP_ECP_MSG), MESSIP_NOTIMEOUT) < 0)
#endif /* !USE_MESSIP_SRR */
		sr_ecp_msg.message (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);   
};

void ecp_mp_sensor::configure_sensor() {
	to_vsp.i_code=VSP_CONFIGURE_SENSOR;
#if !defined(USE_MESSIP_SRR)
	if(write(sd, &to_vsp, sizeof(ECP_VSP_MSG)) == -1)
#else /* USE_MESSIP_SRR */
	int status;
	if(messip_send(ch, 0, 0, &to_vsp, sizeof(ECP_VSP_MSG),
				&status, &from_vsp, sizeof(VSP_ECP_MSG), MESSIP_NOTIMEOUT) < 0)
#endif /* !USE_MESSIP_SRR */
		sr_ecp_msg.message (SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);   
}; // end initiate_sensor

void ecp_mp_sensor::get_reading() {
	get_reading(&image);
};//: get_reading

void ecp_mp_sensor::get_reading(SENSOR_IMAGE* sensor_image) {
	// Sprawdzenie, czy uzyc domyslnego obrazu.
	to_vsp.i_code=VSP_GET_READING;
#if !defined(USE_MESSIP_SRR)
 	if(read(sd, &from_vsp, sizeof(VSP_ECP_MSG))==-1)
#else /* USE_MESSIP_SRR */
	int status;
	if(messip_send(ch, 0, 0, &to_vsp, sizeof(ECP_VSP_MSG),
				&status, &from_vsp, sizeof(VSP_ECP_MSG), MESSIP_NOTIMEOUT) < 0)
#endif /* !USE_MESSIP_SRR */
		sr_ecp_msg.message (SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME);   
	// jesli odczyt sie powodl, przepisanie pol obrazu z bufora komunikacyjnego do image;
	if(from_vsp.vsp_report == VSP_REPLY_OK) {
		memcpy( &(sensor_image->begin), &(from_vsp.comm_image.begin), union_size);
	} else {
		sr_ecp_msg.message ("Reply from VSP not ok");
	}
};//: get_reading
