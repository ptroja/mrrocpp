#if !defined(_ECP_T_MAM_H)
#define _ECP_T_MAM_H

#include "ecp/common/task/ecp_task.h"

// Generator ruchu.
#include "ecp/common/generator/ecp_g_mam.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class mam : public common::task::task
{
private:
	void show_mam_window
#if !defined(USE_MESSIP_SRR)
		(int UI_fd);
#else
		(messip_channel_t *UI_fd);
#endif

	void * UI_communication_thread(void* arg);

public:
	// Kanal komunikacyjny z procesem UI.
#if !defined(USE_MESSIP_SRR)
	name_attach_t * UI_ECP_attach;
#else
	messip_channel_t * UI_ECP_attach;
#endif

	// Obiekt generatora trajektorii.
	generator::manual_moves_automatic_measures *mam_gen;

	// Flaga uzywana do informowania o koncu pracy.
	bool TERMINATE;
	// Flaga uzywana do zatrzymywania/uruchamiania zbierania pomiarow.
	bool START_MEASURES;

	// KONSTRUKTORY
	mam(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
