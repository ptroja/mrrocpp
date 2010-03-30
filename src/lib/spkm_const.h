// -------------------------------------------------------------------------
//                            impconst.h
// Typy i stale wykorzystywane w MRROC++
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

#include <stdint.h>

namespace mrrocpp
{

}


using namespace mrrocpp;


namespace mrrocpp {
namespace lib {


#ifdef __cplusplus
extern "C" {
#endif



#define EDP_SPKM_SECTION "[edp_spkm]"
#define ECP_SPKM_SECTION "[ecp_spkm]"

#define SPKM_NUM_OF_SERVOS	6

#ifdef __cplusplus
}
#endif

} // namespace lib
} // namespace mrrocpp

#endif /* _IMPCONST_H */
