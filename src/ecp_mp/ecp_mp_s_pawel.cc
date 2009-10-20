// ------------------------------------------------------------------------
//                        		ecp_s.cc		dla QNX6.2
//
//                     EFFECTOR CONTROL PROCESS (lib::VSP) - metody klasy ecp_mp_sensor()
//
// Ostatnia modyfikacja: 06.12.2006
// Autor: tkornuta
// ------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#if !defined(USE_MESSIP_SRR)
#include <devctl.h>
#else
#warning file not ported to MESSIP yet
#endif
#include <math.h>

#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ecp_mp/ecp_mp_s_pawel.h"		// zawiera klase ecp_mp_sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
pawel::pawel (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task::task& _ecp_mp_object):
	sensor (_sensor_name, _section_name, _ecp_mp_object) {

	union_size = sizeof(image.sensor_union.ball);

	//  printf("[ecp_mp]\tunion_size = %i\n",union_size);
	//  sr_ecp_msg->message (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME);
}
/************************** CONFIGURE SENSOR ******************************/
void pawel::configure_sensor() {
#if !defined(USE_MESSIP_SRR)
	devmsg.to_vsp.i_code= lib::VSP_CONFIGURE_SENSOR;
//	printf ("[ecp_mp]\tkonfiguracja czujnika\n");
	// Wyslanie polecenia do procesu VSP.
	if (devctl(sd, DEVCTL_RW, &devmsg, sizeof(lib::DEVCTL_MSG), NULL) == 9)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);
#endif
}

/************************** INITIATE  READING *********************************/
void pawel::initiate_reading() {
#if !defined(USE_MESSIP_SRR)
	devmsg.to_vsp.i_code= lib::VSP_INITIATE_READING;
	if (devctl(sd, DEVCTL_RW, &devmsg, sizeof(lib::DEVCTL_MSG), NULL) == 9)
		throw sensor_error(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE);

//	printf("[ecp_mp]\tinitiate reading\n");
#endif
}

/***************************** GET  READING *********************************/
void pawel::get_reading(void) {
#if !defined(USE_MESSIP_SRR)
	if(read(sd, &from_vsp, sizeof(lib::VSP_ECP_MSG)) == -1)
		sr_ecp_msg.message (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME.c_str());

	if(from_vsp.vsp_report == lib::VSP_REPLY_OK)
	{
		memcpy(&image.sensor_union.ball, &from_vsp.comm_image.sensor_union.ball, union_size);

		//int nsec = lround((double)image.sensor_union.ball.ts.tv_nsec/10000000.0);

//		printf ("[nr] %i\t[x] %f\t[y] %f\t[z] %f\n", image.sensor_union.ball.nr, image.sensor_union.ball.x, image.sensor_union.ball.y, image.sensor_union.ball.z);
//		printf ("[ts] %i\n\n", image.sensor_union.ball.ts.tv_sec);

		/*f = fopen("../data/faketrajectory.txt","a");
		fprintf(f,"%i %f %f %f %i\n",image.sensor_union.ball.nr,image.sensor_union.ball.x,image.sensor_union.ball.y,image.sensor_union.ball.z,image.sensor_union.ball.ts.tv_sec);
		fclose(f);*/
	} else {
		printf("[ecp_mp]\treply from VSP not OK\n");
	}
#endif
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
