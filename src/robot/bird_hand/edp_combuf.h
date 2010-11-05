#include <stdint.h>

namespace mrrocpp {
namespace edp {
namespace bird_hand {

const char START_BYTE = '#';

// commands 
const int GET_STATUS = 0x00;
const int GET_ABSPOS = 0x01;
const int SET_SYNCHRO = 0x02;

const int SET_CMD1 = 0x30;
const int SET_CMD2 = 0x31;
const int SET_CMD3 = 0x32;

const int GET_PID = 0x10;
const int GET_LIMIT = 0x11;

const int SET_PID = 0x40;
const int SET_LIMIT = 0x41;

const int HEADER_LEN = 3;

const int LOWER_LIMIT = 0;
const int UPPER_LIMIT = 1;
const int CURRENT_LIMIT = 2;
const int TORQUE_LIMIT = 3;

struct status_
{
	uint8_t status;
	int32_t position;
	int16_t force;
	int16_t current;
	uint16_t abspos;
}__attribute__((__packed__));

struct abspos_
{
	int16_t abspos;
}__attribute__((__packed__));

struct cmd_
{
	int32_t rd;
	uint16_t fd;
	uint16_t b;
	uint16_t t;
}__attribute__((__packed__));

struct pid_
{
	int16_t p;
	int16_t i;
	int16_t d;
}__attribute__((__packed__));

struct limit_
{
	int16_t u_limit;
	int16_t l_limit;
	int16_t cur_limit;
	int16_t f_limit;
}__attribute__((__packed__));

struct synchro_
{
	uint16_t n;
}__attribute__((__packed__));

} // namespace bird_hand
} // namespace edp
} // namespace mrrocpp
