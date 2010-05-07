#if !defined(_TRANSMITTER_H)
#define _TRANSMITTER_H

#include <stdint.h>				// for uint64_t

#include "lib/srlib.h"
#include "player/playerc.h"

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
    TRANSMITTER_PLAYER,
    TRANSMITTER_DRAUGHTSAI,
    TRANSMITTER_GRASPIT
};

enum VA_UNION_VARIANT {RC_WINDOWS,DRAUGHTSAI};

/************ Struktura obrazow czujnika ***************/
typedef struct _TO_VA
{
	VA_UNION_VARIANT command_type;				// polecenie dla VSP

	// wlasciwe pola obrazu - unie!
	union {
		struct {
			char rc_state[54+1];	// 6faces*9facets+trailing '0'
		} rc_windows;

		struct{
			char board[32];
			char player;
		} draughts_ai;

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
		struct {
			char move[25];
			char status;
		} draughts_ai;
		struct {
			double grasp_joint[7];
		} graspit;
		playerc_joystick_t player_joystick;
		playerc_position_t player_position;
		playerc_speech_recognition_t player_speech_recognition;
	};
} FROM_VA;

class transmitter
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
		transmitter (TRANSMITTER_ENUM _transmitter_name, const char* _section_name, task::task& _ecp_mp_object);

		virtual ~transmitter()
		{};

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
				const lib::error_class_t error_class;
				const uint64_t error_no;

				transmitter_error ( lib::error_class_t err_cl, uint64_t err_no) :
					error_class(err_cl), error_no(err_no)
				{
				};
		}; // end: class transmitter_error
}; // end: class transmitter

} // namespace transmitter

typedef std::map<transmitter::TRANSMITTER_ENUM, transmitter::transmitter*> transmitters_t;
typedef transmitters_t::value_type transmitter_item_t;

} // namespace ecp_mp
} // namespace mrrocpp


#endif /* _TRANSMITTER_H */
