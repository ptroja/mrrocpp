
#include <inttypes.h>


#define START_BYTE '#'

// commands
#define COMMAND_MODE_PWM			0x00
#define COMMAND_MODE_CURRENT		0x01
#define COMMAND_MODE_POSITION		0x02

// command params
#define COMMAND_PARAM_SYNCHRO		0x10

// error flags returned by hi::read_write_hardware (defined in servo_gr.h)
#define ALL_RIGHT			0x0000000000000000ULL;
#define SYNCHRO_ZERO		0x0000000000000001ULL;
#define SYNCHRO_SWITCH_ON	0x0000000000000002ULL;
#define LOWER_LIMIT_SWITCH	0x0000000000000004ULL;
#define UPPER_LIMIT_SWITCH	0x0000000000000008ULL;
#define OVER_CURRENT		0x0000000000000010ULL;

struct status_St
{
	uint8_t startByte;
	uint8_t sw1				:1;
	uint8_t sw2				:1;
	uint8_t swSynchr		:1;
	uint8_t synchroZero		:1;
	uint8_t powerStageFault	:1;
	uint8_t overcurrent		:1;
	uint8_t error			:1;
	uint8_t isSynchronized	:1;
	int16_t current;
	int32_t position;
}	__attribute__((__packed__));

struct pwm_St
{
	int16_t pwm;
}	__attribute__((__packed__));


