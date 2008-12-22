#if !defined(_ECP_T_PTEACH_IRP6OT_H)
#define _ECP_T_PTEACH_IRP6OT_H

#include "ecp/common/ecp_task.h"

class ecp_task_pteach_irp6ot: public ecp_task  {
protected:
	// Generator  odtwarzajacy nauczona trajektorie dla celow kalibracji
	ecp_calibration_generator* cg;
	// Warunek, ktorego spelnienie umozliwia realizacje ruchu do nastepnej nauczonej pozycji
	ecp_operator_reaction_condition* orc;
	int pll;                 // liczba nauczonych pozycji
	int i;                // licznik

public:
	// KONSTRUKTORY
	ecp_task_pteach_irp6ot(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

#endif
