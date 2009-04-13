// -------------------------------------------------------------------------
//                            ecp_t_mam.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// 
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
// 
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_T_MAM_H)
#define _ECP_T_MAM_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class mam: public common::task::base  {

public:
	// KONSTRUKTORY
	mam(lib::configurator &_config);
	~mam();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
	void catch_signal(int sig);
	
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
