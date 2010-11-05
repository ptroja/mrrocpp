#if !defined(_ECP_GEN_FESTIVAL_H)
#define _ECP_GEN_FESTIVAL_H

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace festival {
namespace generator {

#define MAX_FESTIVAL_PHRASE	255

#define FESTIVAL_POLISH_VOICE "(voice_cstr_pl_em_diphone)"
#define FESTIVAL_ENGLISH_VOICE "(voice_nitech_us_slt_arctic_hts)"

#define FESTIVAL_SAY_STRING_PREFIX " (SayText \""
#define FESTIVAL_SAY_STRING_SUFFIX "\") (quit)\n"

#define FESTIVAL_CODE_OK "LP\n"
#define FESTIVAL_CODE_ERR "ER\n"
#define FESTIVAL_RETURN_LEN 39

/*
 * polskie znaki w standardzie iso-8859-2 lub odpowiedniki 7-bit:
 * o~ - oun
 * c~ - tsi
 * e~ - eun
 * l/ - eu
 * n~ - eni
 * s~ - esi
 * z~ - ziet
 * z* - rzet
 */

class generator : public common::generator::generator
{
	private:
		int sock;
		
		int read_pending_status;
		ssize_t numread;
		char buf[256];

		int portnum;
		std::string host;
		
		char phrase[MAX_FESTIVAL_PHRASE];
		std::string voice;
		
		int test_mode;	

	public:
		// konstruktor
		generator(common::task::task& _ecp_task);

		virtual bool first_step ();

		virtual bool next_step ();

		enum VOICE {CURRENT_VOICE, POLISH_VOICE, ENGLISH_VOICE};

		char * set_phrase(const char *text);
		bool set_voice(VOICE _voice);
};

}
} // namespace festival
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GEN_FESTIVAL_H */
