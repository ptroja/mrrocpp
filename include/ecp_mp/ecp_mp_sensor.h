#if !defined(_ECP_MP_SENSOR_H)
#define _ECP_MP_SENSOR_H

#include "common/sensor.h"

#include "messip/messip.h"

// TODO: Forward declaration
class ecp_mp_task;

// Klasa bazowa czujnikow po stronie procesu ECP.
class ecp_mp_sensor: public sensor{
protected:
	// Sensor descriptor - uchwyt do /dev/twoj_sensor.
	
	int sd;									
#if defined(USE_MESSIP_SRR)
	messip_channel_t *ch;
#endif /* USE_MESSIP_SRR */
	// Nazwa czujnika.
	char *VSP_NAME;
	
	// Wskaznik na obiekt do komunikacji z SR
	sr_ecp &sr_ecp_msg;

public:

	const SENSOR_ENUM sensor_name; // nazwa czujnika z define w impconst.h
	
	VSP_REPORT vsp_report;

	// Wlasciwy konstruktor czujnika wirtualnego.
	ecp_mp_sensor(SENSOR_ENUM _sensor_name, const char* _section_name, ecp_mp_task& _ecp_mp_object);
	
	virtual void configure_sensor();
	virtual void initiate_reading();
	virtual void terminate();
	virtual void get_reading();
	virtual void get_reading(SENSOR_IMAGE *sensor_image);
};

#endif /* _ECP_MP_SENSOR_H */
