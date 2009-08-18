/*
 * ecp_mp_tr_draughtsAI.h
 *
 *  Created on: Jun 9, 2009
 *      Author: tbem
 */

#ifndef ECP_MP_TR_DRAUGHTSAI_H_
#define ECP_MP_TR_DRAUGHTSAI_H_

#include "ecp_mp/transmitter.h"				// klasa bazowa transmitter

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {


/*==============================STRUCTURES===================================*/

struct game_board_struct{
        char board[32];
        char player;
};

struct result_struct{
        char move[25];
        char status;
};

/*===============================CLASS=======================================*/

class TRDraughtsAI: public transmitter{
	private:									// pola do komunikacji
		//pthread_t worker;
		struct result_struct result;
		struct game_board_struct gameBoard;
		int socketDescriptor;

	public:
		TRDraughtsAI(TRANSMITTER_ENUM _transmitter_name, const char* _section_name, task::task& _ecp_mp_object);
		~TRDraughtsAI(); 	// destruktor czujnika virtualnego
		int AIconnect(const char *host,unsigned short int serverPort);
		int AIdisconnect();
		virtual bool t_read (bool wait);	// odczyt z zawieszaniem lub bez
		virtual bool t_write (void);		// zapis
};

} //namespace mrrocpp
} //namespace ecp_mp
} //namespace transmiter
#endif /* ECP_MP_TR_DRAUGHTSAI_H_ */
