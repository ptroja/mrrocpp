// -------------------------------------------------------------------------
//                                  y_spawn.h
// 
// Plik naglowkowy procesu do spawn'owania po sieci // by Y
// zawiera struktury komunikacyjne
// 
// Ostatnia modyfikacja: czerwiec 2004
// -------------------------------------------------------------------------

#ifndef __Y_SPAWN_H
#define __Y_SPAWN_H

#include <climits>
#include "base/lib/typedefs.h"

namespace mrrocpp {
namespace lib {

typedef struct _my_data { // wiadomosc wysylana do procesu spawnujacego
    msg_header_t hdr;
    int msg_type;
    char node_name[30];
    char binaries_path[PATH_MAX];
    char program_name_and_args[200];
} my_data_t;

typedef struct _my_reply_data { // odpowiedz procesu spawnujacego
    msg_header_t hdr;
    int msg_type;
    int pid; // pid procesu na zdalnym wezle
    int remote_errno; // ew. blad (errno) podczas spawnowania
} my_reply_data_t;

int y_spawn(my_data_t input);// uwaga sciezka ze / na koncu

} // namespace lib
} // namespace mrrocpp

#endif

