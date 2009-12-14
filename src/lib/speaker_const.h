// -------------------------------------------------------------------------
//                            impconst.h
// Typy i stale wykorzystywane w MRROC++
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(_SPEAKER_CONST_H)
#define _SPEAKER_CONST_H

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


#define EDP_SPEAKER_SECTION "[edp_speaker]"
#define ECP_SPEAKER_SECTION "[ecp_speaker]"

#define MAX_TEXT 100 // MAC7
#define MAX_PROSODY 20 // MAC7


#ifdef __cplusplus
}
#endif

} // namespace lib
} // namespace mrrocpp

#endif /* _IMPCONST_H */
