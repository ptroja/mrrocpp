// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - speaker
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/speaker/ecp_r_speaker.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {


// ####################################################################################################
// ####################################################################################################
// ####################################################################################################

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp_robot(lib::ROBOT_SPEAKER, 0, "[edp_speaker]", _config, _sr_ecp)
{
}

robot::robot(common::task::task& _ecp_object) :
	ecp_robot(lib::ROBOT_SPEAKER, 0, "[edp_speaker]", _ecp_object)
{
}

// --------------------------------------------------------------------------
void robot::create_command(void)
{
	// wypelnia bufor wysylkowy do EDP na podstawie danych
	// zawartych w skladowych generatora lub warunku

	ecp_command.instruction.instruction_type = EDP_data.instruction_type;

	switch (EDP_data.instruction_type)
	{
		case lib::SET:
		case lib::SET_GET:
			strcpy(ecp_command.instruction.arm.text_def.text, EDP_data.text);
			strcpy(ecp_command.instruction.arm.text_def.prosody, EDP_data.prosody);
			break;
		case lib::GET:
		case lib::QUERY:
			break;
		case lib::SYNCHRO:
		default: // blad: nieprawidlowe polecenie
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_ECP_COMMAND);
	} // end: switch (instruction_type)
}
// ---------------------------------------------------------------

/*---------------------------------------------------------------------*/
void robot::get_reply(void)
{
	// pobiera z pakietu przeslanego z EDP informacje i wstawia je do
	// odpowiednich skladowych generatora lub warunku
	EDP_data.reply_type = reply_package.reply_type;

	switch (EDP_data.reply_type)
	{
		case lib::ERROR:
			EDP_data.error_no.error0 = reply_package.error_no.error0;
			EDP_data.error_no.error1 = reply_package.error_no.error1;
			break;
		case lib::ACKNOWLEDGE:
			EDP_data.speaking = reply_package.arm.text_def.speaking;
			break;
		default: // bledna przesylka
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_EDP_REPLY);
	} // end: switch (EDP_data.reply_type)
} // end: ecp_speaker_robot::get_reply ()
} // namespace speaker
} // namespace ecp
} // namespace mrrocpp


