#if !defined(_ECP_MAIN_ERROR_H)
#define  _ECP_MAIN_ERROR_H

#include <stdint.h>

class ECP_main_error {  // Klasa obslugi bledow ECP
    public:
      uint64_t error_class;
      uint64_t error_no;

      ECP_main_error ( uint64_t err_cl, uint64_t err_no);
}; // end: class ECP_main_error // by Y&W przerzucone do wnetrza klasy

#endif /* _ECP_MAIN_ERROR_H */
