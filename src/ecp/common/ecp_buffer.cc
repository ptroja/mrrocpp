#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include "common/com_buf.h"
#include "ecp/common/ecp_robot.h"
#include "lib/mis_fun.h"

#include "messip/messip.h"

#if !defined(USE_MESSIP_SRR)
void ecp_buffer::send(int fd)
#else
void ecp_buffer::send (messip_channel_t *ch)
#endif
{
	// Wyslanie do EDP polecenia
	// int command_size;  // rozmiar przesylanej przesylki
	// printf("\n a w send fd=%d",fd);  // debug

	// printf("w send instruction.instruction_type: %d\n", instruction.instruction_type);

	switch (instruction.instruction_type) {
		case SET:
		case SET_GET:
		case GET:
		case SYNCHRO:
		case QUERY:
		case INVALID:
			// command_size = ((BYTE*) (&instruction.address_byte)) - ((BYTE*) (&instruction.instruction_type));
			// by Y bylo command_size zamiast sizeof(in..)
			// by Y&W doszlo  dodatkowe pole w instruction zwiazane z obsluga resource managera

#if !defined(USE_MESSIP_SRR)
			if (MsgSend(fd, &instruction, sizeof(instruction), &reply_package.reply_type, sizeof(r_buffer)) == -1)
#else
			int32_t answer;
			if ( messip_send(ch, 0, 0,
							&instruction, sizeof(instruction),
							&answer, &reply_package.reply_type, sizeof(r_buffer),
							MESSIP_NOTIMEOUT) == -1 )
#endif
			{
				uint64_t e= errno; // kod bledu systemowego
				perror("Send error to EDP_MASTER");
				sr_ecp_msg->message(SYSTEM_ERROR, e, "Send error to EDP_MASTER");
				throw ecp_robot::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
			}

			break;
		default: // blad: nieprawidlowe polecenie
			perror("ECP: INVALID COMMAND TO EDP\n");
			sr_ecp_msg->message(NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
			throw ecp_robot::ECP_error(NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
	}

	set_thread_priority(pthread_self(), MAX_PRIORITY-2);

}

#if !defined(USE_MESSIP_SRR)
void ecp_buffer::query(int fd)
#else
void ecp_buffer::query (messip_channel_t *fd)
#endif
{
	instruction.instruction_type = QUERY;
	send(fd); // czyli wywolanie funkcji ecp_buffer::send, ktora jest powyzej :)
}
