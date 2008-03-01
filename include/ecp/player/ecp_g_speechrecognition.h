#if !defined(_ECP_GEN_SPEECHRECOGNITION_H)
#define _ECP_GEN_SPEECHRECOGNITION_H

#include "ecp/common/ecp_generator.h"
#include "player/playerclient.h"

class speechrecognition_generator : public ecp_generator
{
	private:
		char *hostname;
		int device_index;
		
		PlayerClient *client;
		SpeechRecognitionProxy *device;

	public:
		// konstruktor
		speechrecognition_generator(ecp_task& _ecp_task);
		~speechrecognition_generator();

		virtual bool first_step ();

		virtual bool next_step ();
};

#endif /* _ECP_GEN_SPEECHRECOGNITION_H */
