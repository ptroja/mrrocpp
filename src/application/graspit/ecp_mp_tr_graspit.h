
#ifndef ECP_MP_TR_GRASPIT_H_
#define ECP_MP_TR_GRASPIT_H_

#include "ecp_mp/transmitter/transmitter.h"				// klasa bazowa transmitter

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {


/*==============================STRUCTURES===================================*/

struct result_grasp{
        char move[25];
        char status;
};

/*===============================CLASS=======================================*/

class TRGraspit: public transmitter{
	private:									// pola do komunikacji
		struct result_grasp result;
		int socketDescriptor;

	public:
		TRGraspit(TRANSMITTER_ENUM _transmitter_name, const std::string & _section_name, task::task& _ecp_mp_object);
		~TRGraspit(); 	// destruktor czujnika virtualnego
		void TRconnect(const char *host, uint16_t serverPort);
		void TRdisconnect();
		virtual bool t_read ();	// odczyt z zawieszaniem lub bez
};

} //namespace mrrocpp
} //namespace ecp_mp
} //namespace transmiter
#endif /* ECP_MP_TR_GRASPIT_H_ */
