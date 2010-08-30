/*
 * ecp_mp_tr_draughtsAI.h
 *
 *  Created on: Jun 9, 2009
 *      Author: tbem
 */

#ifndef ECP_MP_TR_DRAUGHTSAI_H_
#define ECP_MP_TR_DRAUGHTSAI_H_

#include "base/ecp_mp/transmitter.h"				// klasa bazowa transmitter
namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

static const std::string TRANSMITTER_DRAUGHTSAI = "TRANSMITTER_DRAUGHTSAI";

/*==============================STRUCTURES===================================*/

struct game_board_struct
{
	char board[32];
	char player;
};

struct result_struct
{
	uint8_t move[25];
	char status;
};

/*===============================CLASS=======================================*/

typedef struct _to_draughts_ai
{
	char board[32];
	char player;
} to_draughts_ai_t;

typedef struct _from_draughts_ai
{
	uint8_t move[25];
	char status;
} from_draughts_ai_t;

typedef transmitter <to_draughts_ai_t, from_draughts_ai_t> DraughtsAI_transmitter_t;

class TRDraughtsAI : public DraughtsAI_transmitter_t
{
private:
	// pola do komunikacji
	struct result_struct result;
	struct game_board_struct gameBoard;
	int socketDescriptor;

public:
	TRDraughtsAI(lib::TRANSMITTER_t _transmitter_name, const char* _section_name, task::task& _ecp_mp_object);
	~TRDraughtsAI(); // destruktor czujnika virtualnego
	void AIconnect(const char *host, unsigned short int serverPort);
	void AIdisconnect();
	virtual bool t_read(); // odczyt z zawieszaniem lub bez
	virtual bool t_write(void); // zapis
};

} //namespace mrrocpp
} //namespace ecp_mp
} //namespace transmiter
#endif /* ECP_MP_TR_DRAUGHTSAI_H_ */
