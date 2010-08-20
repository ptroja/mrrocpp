
#ifndef ECP_MP_TR_GRASPIT_H_
#define ECP_MP_TR_GRASPIT_H_

//#include "lib/sensor_image.h"
#include "base/ecp_mp/transmitter.h" // klasa bazowa transmitter

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {


/*==============================STRUCTURES===================================*/

typedef struct _from_graspit {
	//double grasp_joint[2*6+1]; //tfg
	double grasp_joint[2*6+8]; //bird_hand
} from_graspit_t;

/*===============================CLASS=======================================*/

static const std::string TRANSMITTER_GRASPIT = "TRANSMITTER_GRASPIT";

typedef transmitter<lib::empty_t, from_graspit_t> GraspitTransmitter_t;

class TRGraspit: public GraspitTransmitter_t {
	private:									// pola do komunikacji
		int socketDescriptor;

	public:
		TRGraspit(TRANSMITTER_ENUM _transmitter_name, const char* _section_name, task::task& _ecp_mp_object);
		~TRGraspit(); 	// destruktor czujnika virtualnego
		void TRconnect(const char *host,unsigned short int serverPort);
		void TRdisconnect();
		virtual bool t_read ();	// odczyt z zawieszaniem lub bez
};

} //namespace mrrocpp
} //namespace ecp_mp
} //namespace transmiter
#endif /* ECP_MP_TR_GRASPIT_H_ */
