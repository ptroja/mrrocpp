#if !defined(_ECP_T_PTEACH_IRP6OT_H)
#define _ECP_T_PTEACH_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class pteach: public common::task::task  {
protected:
	// Generator  odtwarzajacy nauczona trajektorie dla celow kalibracji
	common::generator::calibration* cg;
	// Warunek, ktorego spelnienie umozliwia realizacje ruchu do nastepnej nauczonej pozycji
	common::operator_reaction_condition* orc;
	int pll;                 // liczba nauczonych pozycji
	int i;                // licznik

public:
	// KONSTRUKTORY
	pteach(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
