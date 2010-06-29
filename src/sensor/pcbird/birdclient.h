#ifndef __BIRDCLIENT_H
#define __BIRDCLIENT_H

#include <inttypes.h>

namespace mrrocpp {
namespace ecp_mp {

// struktura z pozycja i katami pcbirda
typedef struct {
    float x, y, z;	// pozycja
    float a, b, g;	// katy (a = azimuth, b = elevation, g = roll)
    float distance;	// odleglosc
    uint32_t ts_sec, ts_usec;	// timestamp
} pcbird_pos_t;

// polaczenie z serwerem PCBird, zwraca deskryptor socketa
int pcbird_connect(const char *addr, unsigned short port);

// koniec sesji PCBird
void pcbird_disconnect(int fd);

// rozpoczyna streaming pozycji
int pcbird_start_streaming(int fd);

// konczy streaming pozycji
int pcbird_stop_streaming(int fd);

// pojedynczy odczyt pozycji
int pcbird_get_single_position(int fd, pcbird_pos_t *p);

// czy dane do gniazda nadeszly? (uzywane przy streamingu)
int pcbird_data_avail(int fd);

// nieblokujacy odczyt pozycji w trybie streaming
int pcbird_get_streaming_position(int fd, pcbird_pos_t *p);

} // namespace ecp_mp
} // namespace mrrocpp

#endif

