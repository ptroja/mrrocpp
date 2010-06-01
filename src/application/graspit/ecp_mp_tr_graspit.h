
#ifndef ECP_MP_TR_GRASPIT_H_
#define ECP_MP_TR_GRASPIT_H_

#include "lib/sensor.h"
#include "ecp_mp/transmitter/transmitter.h"				// klasa bazowa transmitter

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {


/*==============================STRUCTURES===================================*/

struct result_grasp
{
	char move[25];
	char status;
};

typedef struct _from_graspit {
	double grasp_joint[7];
} from_graspit_t;

/*===============================CLASS=======================================*/

static const std::string TRANSMITTER_GRASPIT = "TRANSMITTER_GRASPIT";

typedef transmitter<lib::empty_t, from_graspit_t> GraspitTransmitter_t;

class TRGraspit: public GraspitTransmitter_t {
	private:									// pola do komunikacji
		struct result_grasp result;
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
