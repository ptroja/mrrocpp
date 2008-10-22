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
#include "ecp/common/ecp_task.h"
//#include "ecp_mp/ecp_mp_t_fsautomat.h"
#include "mp/State.h"
#include "mp/CubeState.h"
#include "mp/SingleManipulation.h"

class mp_task_fsautomat : public mp_task  
{
	protected:
		bool break_state;
    	CubeState *cube_state;
		// should depend on init node in xml task definition or be computed in Condition
    	bool manipulation_sequence_computed;

	public:
		// stl'owa lista manipulacji
		std::list<SingleManipulation> manipulation_list;
		
	    // konstruktor
	    mp_task_fsautomat(configurator &_config);
	
	    ~mp_task_fsautomat();

		// methods for mp template
		void task_initialization(void);
		void main_task_algorithm(void);
		
		std::list<State> *takeStatesList(void);
		std::map<char *, State, ecp_task::str_cmp> * takeStatesMap();
		bool executeMotion(State &state);
		bool runEmptyGenForSet(State &state);
		bool runEmptyGen(State &state);
		bool runWaitFunction(State &state);
		bool stopProperGen(State &state);
		bool sensorInitialization();
		bool initializeCubeState(State &state);
		bool initiateSensorReading(State &state);
		bool getSensorReading(State &state);
		bool writeCubeState(State &state);
		bool changeCubeState(State &state);
		bool communicate_with_windows_solver(State &state);
		bool translateManipulationSequence(StateHeap &sh);

		void configureProperSensor(char *propSensor);
		void configureProperTransmitter(char *propTrans);

}; // end : class mp_task_fsautomat
#endif
