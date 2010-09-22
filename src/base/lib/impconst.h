/*!
 * @file impconst.h
 * @brief Fundamental types and constants used.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 *
 * @ingroup LIB
 */

#if !defined(_IMPCONST_H)
#define _IMPCONST_H

#include <string>
#include <stdint.h>

namespace mrrocpp {
}
using namespace mrrocpp;
namespace mrrocpp {
namespace lib {

// Rozmiary buforow
const int MP_2_ECP_NEXT_STATE_STRING_SIZE = 100;
const int MP_2_ECP_STRING_SIZE = 300;
const int ECP_2_MP_STRING_SIZE = 300;
const int MAX_TEXT = 100; // MAC7
const int MAX_PROSODY = 20; // MAC7


// Stale do komunikacji


const int CONNECT_RETRY = 200;
const int CONNECT_DELAY = 50;

// ----------------------- PRZYDATNE STALE ---------------------------
typedef double frame_tab[3][4];

const std::string MP_SECTION = "[mp]";
const std::string UI_SECTION = "[ui]";

typedef std::string robot_name_t;
typedef std::string TRANSMITTER_t;

const robot_name_t ROBOT_UNDEFINED = "ROBOT_UNDEFINED";

// Other robots in dedicated robot consts files

const int MAX_SERVOS_NR = 8;

const double EDP_STEP = 0.002; // Krok sterowania w [s]

// dla starej wersji sterowania
//#define FORCE_INERTIA 0.96
//#define TORQUE_INERTIA 0.98
//#define FORCE_RECIPROCAL_DAMPING -0.00025
//#define TORQUE_RECIPROCAL_DAMPING -0.005

// wartosci podstawowe dla sterowania silowego
const double FORCE_INERTIA = 20;
const double TORQUE_INERTIA = 0.5;
const double FORCE_RECIPROCAL_DAMPING = 0.005;
const double TORQUE_RECIPROCAL_DAMPING = 0.1;

const std::string ROBOT_TEST_MODE = "robot_test_mode";
const std::string FORCE_SENSOR_TEST_MODE = "force_sensor_test_mode";

// Stale czasowe

const int QNX_MAX_PRIORITY = 50;

// STALE PULSOW MP, ECP, READER

#define MP_START (_PULSE_CODE_MINAVAIL + 1)
#define MP_STOP (_PULSE_CODE_MINAVAIL + 2)
#define MP_PAUSE (_PULSE_CODE_MINAVAIL + 3)
#define MP_RESUME (_PULSE_CODE_MINAVAIL + 4)
#define MP_TRIGGER (_PULSE_CODE_MINAVAIL + 5)

#define MP_TO_ECP_COMMUNICATION_REQUEST (_PULSE_CODE_MINAVAIL + 6)

#define ECP_TRIGGER (_PULSE_CODE_MINAVAIL + 1)

#define READER_START (_PULSE_CODE_MINAVAIL + 1)
#define READER_STOP (_PULSE_CODE_MINAVAIL + 2)
#define READER_TRIGGER (_PULSE_CODE_MINAVAIL + 3)

#define ECP_WAIT_FOR_START (_PULSE_CODE_MINAVAIL + 2)
#define ECP_WAIT_FOR_STOP (_PULSE_CODE_MINAVAIL + 3)
#define ECP_WAIT_FOR_COMMAND (_PULSE_CODE_MINAVAIL + 4)
#define ECP_WAIT_FOR_NEXT_STATE (_PULSE_CODE_MINAVAIL + 5)

} // namespace lib
} // namespace mrrocpp

#endif /* _IMPCONST_H */
