#if !defined(_ECP_MAIN_ERROR_H)
#define  _ECP_MAIN_ERROR_H

#include "lib/com_buf.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ECP_main_error
{ // Klasa obslugi bledow ECP
public:
	const lib::error_class_t error_class;
	const uint64_t error_no;

	ECP_main_error(lib::error_class_t err_cl, uint64_t err_no);
}; // end: class ECP_main_error // by Y&W przerzucone do wnetrza klasy

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_MAIN_ERROR_H */
