#ifndef __HI_MOXA_COMBUF_H
#define __HI_MOXA_COMBUF_H

#include <stdint.h>

namespace mrrocpp {
namespace edp {
namespace hi_moxa {

#define SERVO_ST_BUF_LEN 30
const char START_BYTE = '#';

// commands
const int COMMAND_MODE_PWM = 0x00;
const int COMMAND_MODE_CURRENT = 0x01;
const int COMMAND_MODE_POSITION = 0x02;
const int COMMAND_SET_PARAM = 0x0f;

// command parameters
const int COMMAND_PARAM_SYNCHRO = 0x10;

// SET_PARAM command parameters
const int PARAM_SYNCHRONIZED = 0x10;
const int PARAM_MAXCURRENT = 0x20;
const int PARAM_PID_POS_P = 0x30;
const int PARAM_PID_POS_I = 0x40;
const int PARAM_PID_POS_D = 0x50;
const int PARAM_PID_CURR_P = 0x60;
const int PARAM_PID_CURR_I = 0x70;
const int PARAM_PID_CURR_D = 0x80;
const int PARAM_DRIVER_MODE = 0x90;

// error flags returned by hi::read_write_hardware (defined in servo_gr.h)
const uint64_t ALL_RIGHT = 0x0000000000000000ULL;
const uint64_t SYNCHRO_ZERO = 0x0000000000000001ULL;
const uint64_t SYNCHRO_SWITCH_ON = 0x0000000000000002ULL;
const uint64_t LOWER_LIMIT_SWITCH = 0x0000000000000004ULL;
const uint64_t UPPER_LIMIT_SWITCH = 0x0000000000000008ULL;
const uint64_t OVER_CURRENT = 0x0000000000000010ULL;

struct status_St {
	uint8_t startByte;
	uint8_t sw1 :1;
	uint8_t sw2 :1;
	uint8_t swSynchr :1;
	uint8_t synchroZero :1;
	uint8_t powerStageFault :1;
	uint8_t overcurrent :1;
	uint8_t error :1;
	uint8_t isSynchronized :1;
	int16_t current;
	int32_t position;
}__attribute__((__packed__));

struct pwm_St {
	int16_t pwm;
}__attribute__((__packed__));

union param_Un {
	int8_t synchronized;
	int8_t driver_mode;
	int16_t maxcurrent;
	int16_t pid_coeff;
	int16_t largest;
};

struct servo_St{
	char buf[SERVO_ST_BUF_LEN];
	uint8_t command_params;
	struct status_St drive_status;
//	int32_t position_offset;
	int32_t current_absolute_position;
	int32_t previous_absolute_position;
	double current_position_inc;
	bool first_hardware_read;
	bool trace_resolver_zero;
}__attribute__((__packed__));

} // namespace hi_moxa
} // namespace edp
} // namespace mrrocpp

#endif // __HI_MOXA_COMBUF_H
