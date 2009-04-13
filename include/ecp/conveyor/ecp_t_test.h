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

#if !defined(_ECP_T_CONV_TEST_H)
#define _ECP_T_CONV_TEST_H

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_task.h"
#include "ecp/conveyor/ecp_local.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace task {

class ecp_task_conveyor_test: public common::task::ecp_task 
{

public:
	// KONSTRUKTORY
	ecp_task_conveyor_test(configurator &_config);
	~ecp_task_conveyor_test();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
