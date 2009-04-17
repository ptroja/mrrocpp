#if !defined(_TRANSMITTER_H)
#define _TRANSMITTER_H

#include <stdint.h>				// for uint64_t

#include "player/playerc.h"

#include "lib/srlib.h"

namespace mrrocpp {
namespace ecp_mp {
namespace task {
// XXX Forward declaration
class task;
}
}
}


namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {




// TRASMITERY

enum TRANSMITTER_ENUM {
    TRANSMITTER_UNDEFINED,
    TRANSMITTER_RC_WINDOWS,
    TRANSMITTER_PLAYER
};

enum VA_UNION_VARIANT {RC_WINDOWS};

/************ Struktura obrazow czujnika ***************/
typedef struct _TO_VA
{
	VA_UNION_VARIANT command_type;				// polecenie dla VSP

	// wlasciwe pola obrazu - unie!
	union {
		struct {
			char rc_state[54];
		} rc_windows;
	};
} TO_VA;


/************ Struktura obrazow czujnika ***************/
typedef struct _FROM_VA
{
	VA_UNION_VARIANT reply_type;				// polecenie dla VSP

	// wlasciwe pola obrazu - unie!
	union {
		struct {
			char sequence[100];
		} rc_windows;
		playerc_joystick_t player_joystick;
		playerc_position_t player_position;
		playerc_speech_recognition_t player_speech_recognition;
	};
} FROM_VA;

class base
{
		// Klasa bazowa dla transmitterow (klasa abstrakcyjna)
		// Transmittery konkretne wyprowadzane sa z klasy bazowej
	public:

		const TRANSMITTER_ENUM transmitter_name; // nazwa czujnika z define w impconst.h

		// Bufor nadawczy
		TO_VA to_va;
		// Bufor odbiorczy
		FROM_VA from_va;

	protected:
		// Wskaznik na obiekt do komunikacji z SR
		lib::sr_ecp &sr_ecp_msg;

	public:
		base (TRANSMITTER_ENUM _transmitter_name, const char* _section_name, task::task& _ecp_mp_object);

		virtual ~base()
		{}
		;

		// odczyt z zawieszaniem lub bez
		virtual bool t_read (bool wait)
		{
			return true;
		};
		// zapis
		virtual bool t_write (void)
		{
			return true;
		};

		class transmitter_error
		{  // Klasa obslugi bledow czujnikow
			public:
				uint64_t error_class;
				uint64_t error_no;

				transmitter_error ( uint64_t err_cl, uint64_t err_no)
				{
					error_class = err_cl;
					error_no = err_no;
				};
		}
		; // end: class transmitter_error
}
; // end: class transmitter


} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp


#endif /* _TRANSMITTER_H */
