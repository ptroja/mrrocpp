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

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/task/ecp_task.h"
#include "ecp/conveyor/ecp_r_conv.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace task {

class test: public common::task::task
{

public:
	// KONSTRUKTORY
	test(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
