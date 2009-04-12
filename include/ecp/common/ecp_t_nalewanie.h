// -------------------------------------------------------------------------
//                            ecp_t_nalewanie.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
//
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
//
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_T_NALEWANIE_H)
#define _ECP_T_NALEWANIE_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_task_nalewanie: public common::ecp_task  {
protected:
	ecp_smooth_generator* sg;

public:
	// KONSTRUKTORY
	ecp_task_nalewanie(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
