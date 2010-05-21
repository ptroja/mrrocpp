
#include <inttypes.h>


#define START_BYTE '#'

// commands 
#define GET_STATUS	0x00
#define GET_ABSPOS	0x01
#define SET_SYNCHRO	0x02

#define SET_CMD1	0x30
#define SET_CMD2	0x31
#define SET_CMD3	0x32

#define GET_PID		0x10
#define GET_LIMIT	0x11

#define SET_PID		0x40
#define SET_LIMIT	0x41

#define HEADER_LEN 3


struct status_
{
	uint8_t status;
	int32_t position;
	int16_t force;
	int16_t current;
}	__attribute__((__packed__));

struct abspos_
{
	int16_t abspos;
}	__attribute__((__packed__));


struct cmd_
{
	int32_t rd;
	uint16_t fd;
	uint16_t b;
	uint16_t t;
}	__attribute__((__packed__));

struct pid_
{
	int16_t p;
	int16_t i;
	int16_t d;
}	__attribute__((__packed__));

struct limit_
{
	int16_t u_limit;
	int16_t l_limit;
	int16_t cur_limit;
	int16_t f_limit;
}	__attribute__((__packed__));

struct synchro_
{
	uint16_t n;
}	__attribute__((__packed__));
