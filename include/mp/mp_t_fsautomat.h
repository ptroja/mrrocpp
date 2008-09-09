// -------------------------------------------------------------------------
//                            mp_t_fsautomat.h
// Definicje struktur danych i metod dla procesow MP - Finite State Automaton
// Wersja z generatorami uruchaminami na poziomie ECP
// Autor:						Marek Kisiel
// Ostatnia modyfikacja: 	2008
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_FSAUTOMAT)
#define __MP_TASK_FSAUTOMAT

#include "mp/mp.h"
//#include "ecp_mp/ecp_mp_t_fsautomat.h"
#include "State.h"

class mp_task_fsautomat : public mp_task  
{
	protected:
		bool break_state;

	public:
	    // konstruktor
	    mp_task_fsautomat(configurator &_config);
	
	    ~mp_task_fsautomat();

		// methods for mp template
		void task_initialization(void);
		void main_task_algorithm(void);
		
		std::list<State> *takeStatesList(void);
		bool executeMotion(State &state);
		bool runEmptyGenForSet(State &state);
		bool runEmptyGen(State &state);

}; // end : class mp_task_fsautomat
#endif
