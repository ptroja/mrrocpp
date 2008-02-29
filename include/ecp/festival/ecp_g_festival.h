#if !defined(_ECP_GEN_FESTIVAL_H)
#define _ECP_GEN_FESTIVAL_H

#include "ecp/common/ecp_generator.h"

#define MAX_FESTIVAL_PHRASE	255

#define FESTIVAL_POLISH_VOICE "(voice_cstr_pl_em_diphone)"
#define FESTIVAL_ENGLISH_VOICE "(voice_nitech_us_slt_arctic_hts)"

#define FESTIVAL_SAY_STRING_PREFIX " (SayText \""
#define FESTIVAL_SAY_STRING_SUFFIX "\") (quit)\n"

#define FESTIVAL_CODE_OK "LP\n"
#define FESTIVAL_CODE_ERR "ER\n"
#define FESTIVAL_RETURN_LEN 39

class festival_generator : public ecp_generator
{
	private:
		int sock;
		
		int read_pending_status;
		int numread;
		char buf[256];

		int portnum;
		char *host;
		
		char phrase[MAX_FESTIVAL_PHRASE];
		char *voice;
		
		int test_mode;	

	public:
		// konstruktor
		festival_generator(ecp_task& _ecp_task);
		~festival_generator();

		virtual bool first_step ();

		virtual bool next_step ();

		enum VOICE {CURRENT_VOICE, POLISH_VOICE, ENGLISH_VOICE};

		char * set_phrase(const char *text);
		bool set_voice(VOICE _voice);
};

#endif /* _ECP_GEN_FESTIVAL_H */
