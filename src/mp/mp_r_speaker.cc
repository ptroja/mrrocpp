// -------------------------------------------------------------------------
//                              mp.cc
// 
// MP Master Process - methods
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp_r_speaker.h"




// DLA ROBOTA SPEAKER
  mp_speaker_robot::mp_speaker_robot (mp_task* mp_object_l) :
	 mp_robot (ROBOT_SPEAKER, "[ecp_speaker]", mp_object_l) {}; // Konstruktor

// --------------------------------------------------------------------------
void mp_speaker_robot::create_next_pose_command (void) {
// wypelnia bufor wysylkowy do ECP na podstawie danych
// zawartych w skladowych generatora lub warunku
	
	mp_command.mp_package.instruction.instruction_type = ecp_td.instruction_type;

	switch (ecp_td.instruction_type) {
		case SET:
		case SET_GET:
			// Wypelniamy czesc zwiazana z polozeniem ramienia
			strcpy(mp_command.mp_package.instruction.arm.text_def.text, ecp_td.text);
			strcpy(mp_command.mp_package.instruction.arm.text_def.prosody, ecp_td.prosody);
			break;
		case GET:
		case QUERY:
			break;
		case SYNCHRO:
		default: // blad: nieprawidlowe polecenie
			throw MP_error (NON_FATAL_ERROR, INVALID_ECP_COMMAND);
	}
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------------
void mp_speaker_robot::get_reply (void) {
  // pobiera z pakietu przeslanego z ECP informacje i wstawia je do
  // odpowiednich skladowych generatora lub warunku
	
	ecp_td.ecp_reply = ecp_reply.reply;
	ecp_td.reply_type = ecp_reply.ecp_reply.reply_package.reply_type;

	switch (ecp_td.reply_type) {
		case ERROR:
			ecp_td.error_no.error0 = ecp_reply.ecp_reply.reply_package.error_no.error0;
			ecp_td.error_no.error1 = ecp_reply.ecp_reply.reply_package.error_no.error1;
		break;
		case ACKNOWLEDGE:
			ecp_td.speaking = ecp_reply.ecp_reply.reply_package.arm.text_def.speaking;
		break;
		default:  // bledna przesylka
			throw MP_error (NON_FATAL_ERROR, INVALID_EDP_REPLY);
	} // end: switch (reply_type)
} // end: mp_speaker_robot::get_reply (robot& r)
// --------------------------------------------------------------------------
