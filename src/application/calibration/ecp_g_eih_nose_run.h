// -------------------------------------------------------------------------
//                            generator/ecp_g_force.h dla QNX6
// Deklaracje generatorow dla procesow ECP z wykorzystaniem sily
//
// -------------------------------------------------------------------------


#if !defined(_ECP_GEN_FORCE_H)
#define _ECP_GEN_FORCE_H

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/generator/ecp_g_teach_in.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "lib/mrmath/mrmath.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// Generator do wykrywania zmiany wagi chwytaka wraz z obiektem chwytanym
// ciezary wyskalowane w newtonach
#define WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE 10
#define USLEEP_TIME 10000

// --------------------------------------------------------------------------
// Generator trajektorii dla zadania kalibracji ukladu eih.
// Rozni sie od tff_nose_run tym ze zatrzymuje sie po chwili i trzeba go uzywac w petli

class eih_nose_run : public tff_nose_run
{
	int count;

public:
    // konstruktor
    eih_nose_run(common::task::task& _ecp_task, int step=0);

    virtual bool next_step ();

}; // end : class ecp_eih_nose_run_generator


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
