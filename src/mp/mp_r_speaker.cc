#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp_r_speaker.h"

namespace mrrocpp {
namespace mp {
namespace common {

speaker_robot::speaker_robot (task::base &mp_object_l) :
		robot (ROBOT_SPEAKER, "[ecp_speaker]", mp_object_l)
{}

void speaker_robot::create_next_pose_command (void)
{
	// wypelnia bufor wysylkowy do ECP na podstawie danych
	// zawartych w skladowych generatora lub warunku

	mp_command.instruction.instruction_type = ecp_td.instruction_type;

	switch (ecp_td.instruction_type) {
		case lib::SET:
		case lib::SET_GET:
			// Wypelniamy czesc zwiazana z polozeniem ramienia
			strcpy(mp_command.instruction.arm.text_def.text, ecp_td.text);
			strcpy(mp_command.instruction.arm.text_def.prosody, ecp_td.prosody);
			break;
		case lib::GET:
		case lib::QUERY:
			break;
		case lib::SYNCHRO:
		default: // blad: nieprawidlowe polecenie
			throw MP_error (lib::NON_FATAL_ERROR, INVALID_ECP_COMMAND);
	}
}

void speaker_robot::get_reply(void)
{
	// pobiera z pakietu przeslanego z ECP informacje i wstawia je do
	// odpowiednich skladowych generatora lub warunku

	ecp_td.ecp_reply = ecp_reply_package.reply;
	ecp_td.reply_type = ecp_reply_package.reply_package.reply_type;

	switch (ecp_td.reply_type) {
		case lib::ERROR:
			ecp_td.error_no.error0
					= ecp_reply_package.reply_package.error_no.error0;
			ecp_td.error_no.error1
					= ecp_reply_package.reply_package.error_no.error1;
			break;
		case lib::ACKNOWLEDGE:
			ecp_td.speaking
					= ecp_reply_package.reply_package.arm.text_def.speaking;
			break;
		default: // bledna przesylka
			throw MP_error (lib::NON_FATAL_ERROR, INVALID_EDP_REPLY);
	}
}

} // namespace common
} // namespace mp
} // namespace mrrocpp

