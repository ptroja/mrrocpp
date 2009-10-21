// -------------------------------------------------------------------------
//                            impconst.h
// Typy i stale wykorzystywane w MRROC++
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(_IMPCONST_H)
#define _IMPCONST_H

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



#define CONNECT_RETRY	200
#define CONNECT_DELAY	50

// ----------------------- PRZYDATNE STALE ---------------------------
typedef double frame_tab[3][4];

#ifndef ABS
#define ABS(x) (((x)<0)?-(x):(x))   // wartosc bezwzgledna 'x'
#endif

// by Y - ROBOTY

typedef enum _ROBOT_ENUM {
	ROBOT_UNDEFINED,
	ROBOT_IRP6_ON_TRACK,
	ROBOT_IRP6_POSTUMENT,
	ROBOT_CONVEYOR,
	ROBOT_SPEAKER,
	ROBOT_IRP6_MECHATRONIKA,
	ROBOT_POLYCRANK,
	ROBOT_ELECTRON,
	ROBOT_FESTIVAL,
	ROBOT_HAND,
	ROBOT_SPEECHRECOGNITION
} robot_name_t;

enum FORCE_SENSOR_ENUM {
	FORCE_SENSOR_ATI3084,
	FORCE_SENSOR_ATI6284
};


#define MAX_SERVOS_NR 8

// Liczba osi (stopni swobody)
#define IRP6_ON_TRACK_NUM_OF_SERVOS	8
#define IRP6_POSTUMENT_NUM_OF_SERVOS	7
#define IRP6_MECHATRONIKA_NUM_OF_SERVOS	5
#define POLYCRANK_NUM_OF_SERVOS	8
#define CONVEYOR_NUM_OF_SERVOS		1
#define HAND_NUM_OF_SERVOS		2

#define MAX_TEXT 100 // MAC7
#define MAX_PROSODY 20 // MAC7

#define TIME_SLICE 500000 // by Y

#define STEP              0.002  // Krok sterowania w [s]



// dla starej wersji sterowania
//#define FORCE_INERTIA 0.96
//#define TORQUE_INERTIA 0.98
//#define FORCE_RECIPROCAL_DAMPING -0.00025
//#define TORQUE_RECIPROCAL_DAMPING -0.005

// wartosci podstawowe dla sterowania silowego
#define FORCE_INERTIA 20
#define TORQUE_INERTIA 0.5
#define FORCE_RECIPROCAL_DAMPING 0.005
#define TORQUE_RECIPROCAL_DAMPING 0.1


// Stale czasowe
#define ONE_MSEC  1000000L // 1 milisek w nanosekundach
#define TEN_MSEC 10000000L // 10 milisek w nanosekundach

#define MAX_PRIORITY    50

// STALE PUSLOW MP, ECP, READER

#define MP_START (_PULSE_CODE_MINAVAIL + 1)
#define MP_STOP (_PULSE_CODE_MINAVAIL + 2)
#define MP_PAUSE (_PULSE_CODE_MINAVAIL + 3)
#define MP_RESUME (_PULSE_CODE_MINAVAIL + 4)
#define MP_TRIGGER (_PULSE_CODE_MINAVAIL + 5)

#define ECP_TRIGGER (_PULSE_CODE_MINAVAIL + 1)

#define READER_START (_PULSE_CODE_MINAVAIL + 1)
#define READER_STOP (_PULSE_CODE_MINAVAIL + 2)
#define READER_TRIGGER (_PULSE_CODE_MINAVAIL + 3)

#define ECP_WAIT_FOR_START (_PULSE_CODE_MINAVAIL + 2)
#define ECP_WAIT_FOR_STOP (_PULSE_CODE_MINAVAIL + 3)
#define ECP_WAIT_FOR_COMMAND (_PULSE_CODE_MINAVAIL + 4)
#define ECP_WAIT_FOR_NEXT_STATE (_PULSE_CODE_MINAVAIL + 5)


const uint64_t ALL_RIGHT                           = 0x0000000000000000ULL;
const uint64_t SYNCHRO_ZERO                        = 0x0000000000000001ULL;
const uint64_t SYNCHRO_SWITCH_ON                   = 0x0000000000000002ULL;
const uint64_t SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO  = 0x0000000000000003ULL;
const uint64_t LOWER_LIMIT_SWITCH                  = 0x0000000000000004ULL;
const uint64_t UPPER_LIMIT_SWITCH                  = 0x0000000000000008ULL;
const uint64_t OVER_CURRENT                        = 0x0000000000000010ULL;

const uint64_t HARDWARE_ERROR_MASK = 0xE739CCE739CE739CULL;
const uint64_t MASK_RESOLVER_ZERO = 0x3F7BDEF7BDEF7BDEULL;

#define MP_2_ECP_STRING_SIZE	100

#ifdef __cplusplus
}
#endif

} // namespace lib
} // namespace mrrocpp

#endif /* _IMPCONST_H */
