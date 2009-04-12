#if !defined(_ECP_GEN_SPEECHRECOGNITION_H)
#define _ECP_GEN_SPEECHRECOGNITION_H

#include "ecp/common/ecp_generator.h"
#include "player/playerclient.h"

namespace mrrocpp {
namespace ecp {
namespace player {

class speechrecognition_generator : public common::ecp_generator
{
	private:
		PlayerClient *client;
		SpeechRecognitionProxy *device;

	public:
		// konstruktor
		speechrecognition_generator(common::ecp_task& _ecp_task);
		~speechrecognition_generator();

		virtual bool first_step ();

		virtual bool next_step ();
};

} // namespace player
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GEN_SPEECHRECOGNITION_H */
