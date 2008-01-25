// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie powielania rysunku
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_KD_H)
#define __MP_TASK_KD_H

#include "mp/mp.h"

class mp_task_kd: public mp_task  {
protected:
	bool break_state;
	
public:

	// KONSTRUKTORY
	mp_task_kd(void);
	~mp_task_kd(void);
	
	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);
  
};

#endif
