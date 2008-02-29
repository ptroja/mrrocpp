#if !defined(_ECP_GEN_FESTIVAL_H)
#define _ECP_GEN_FESTIVAL_H

#include "ecp/common/ecp_generator.h"

#define MAX_FESTIVAL_PHRASE	255

#define FESTIVAL_SAY_STRING_PREFIX "(voice_cstr_pl_em_diphone) (SayText \""
#define FESTIVAL_SAY_STRING_SUFFIX "\")\n"

#define FESTIVAL_QUIT_STRING "(quit)\n"

#define FESTIVAL_CODE_OK "LP\n"
#define FESTIVAL_CODE_ERR "ER\n"
#define FESTIVAL_RETURN_LEN 39

class festival_generator : public ecp_generator
{
	private:
	    	int sock;
		char phrase[MAX_FESTIVAL_PHRASE];
		bool read_pending;
		int numread;
		char buf[256];

		int portnum;
		char *host;

	public:
		// konstruktor
		festival_generator(ecp_task& _ecp_task);

		virtual bool first_step ();

		virtual bool next_step ();

		char * set_phrase(const char *text);

};

#endif /* _ECP_GEN_FESTIVAL_H */
