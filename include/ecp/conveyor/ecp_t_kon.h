// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// 
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
// 
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_T_CONV_KON_H)
#define _ECP_T_CONV_KON_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace task {

class kon: public common::task::task  {

public:
	// KONSTRUKTORY
	kon(lib::configurator &_config);
	~kon();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
