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
#include "ecp_mp/ecp_mp_task.h"
//#include "ecp_mp/ecp_mp_t_fsautomat.h"
#include "mp/State.h"
#include "mp/CubeState.h"
#include "mp/SingleManipulation.h"

namespace mrrocpp {
namespace mp {
namespace task {

class fsautomat : public task
{
	protected:
		bool break_state;
    	common::CubeState *cube_state;
		// should depend on init node in xml task definition or be computed in Condition
    	bool manipulation_sequence_computed;

	public:
		// stl'owa lista manipulacji
		std::list<common::SingleManipulation> manipulation_list;

	    // konstruktor
	    fsautomat(lib::configurator &_config);

		// methods for mp template
		void task_initialization(void);
		void main_task_algorithm(void);

		std::list<common::State> *takeStatesList(void);
		common::State * createState(xmlNode *stateNode);
		std::map<const char *, common::State, ecp_mp::task::task::str_cmp> * takeStatesMap();
		void executeMotion(common::State &state);
		void runEmptyGenForSet(common::State &state);
		void runEmptyGen(common::State &state);
		void runWaitFunction(common::State &state);
		void stopProperGen(common::State &state);
		void sensorInitialization();
		void initializeCubeState(common::State &state);
		void initiateSensorReading(common::State &state);
		void getSensorReading(common::State &state);
		void writeCubeState(common::State &state);
		void changeCubeState(common::State &state);
		void changeCubeState(int turn_angle);
		void communicate_with_windows_solver(common::State &state);
		void translateManipulationSequence(common::StateHeap &sh);

		void configureProperSensor(const char *propSensor);
		void configureProperTransmitter(const char *propTrans);

}; // end : class mp_task_fsautomat

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
