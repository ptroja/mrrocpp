#ifndef MP_ROBOT_H_
#define MP_ROBOT_H_

#include <boost/shared_ptr.hpp>

#include "base/mp/mp_task.h"
#include "base/ecp_mp/ecp_mp_robot.h"
#include "base/mp/mp_generator.h"

#include "lib/agent/RemoteAgent.h"
#include "lib/agent/RemoteBuffer.h"

#include <ctime>
#include <sys/types.h>

namespace mrrocpp {
namespace mp {
namespace robot {

class robot : public ecp_mp::robot
{
	friend class mp::generator::generator;

	/**
	 * This flag is set when the new data has arrived from the robot.
	 *
	 * This flag belongs to the internal memory of the agent ($c_{c_j}$)
	 * and is supposed to make transition function implementation easier.
	 */
	bool new_data_flag;

protected:
	task::task &mp_object;

public:
	//! Bufor z odpowiedzia z ECP
	lib::ECP_REPLY_PACKAGE ecp_reply_package;
	DataBuffer<lib::ECP_REPLY_PACKAGE> ecp_reply_package_buffer;

	//! Returns true if new data was received since last call
	bool is_new_data();

private:
#if !defined(PROCESS_SPAWN_RSH)
	//! deskryptor wezla na ktorym jest powolane ECP oraz jego PID
	uint32_t nd;
#endif

	const pid_t ECP_pid;

protected:
	//! ECP agent
	RemoteAgent ecp_agent;

	//! ECP command buffer
	RemoteBuffer<lib::MP_COMMAND_PACKAGE> mp_command_buffer;

public:
	//! Bufor z rozkazem dla ECP
	lib::MP_COMMAND_PACKAGE mp_command;

	//! okresla czy robot ma byc obslugiwany w Move
	bool communicate;

	//! obiekt do komunikacji z SR
	lib::sr_ecp &sr_ecp_msg;

	//! ew. koordynacja ciagla domyslnie wylaczona ma wplyw na instrukcje move
	bool continuous_coordination;

	robot(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l);

	virtual ~robot();

	// Zlecenie wykonania ruchu przez robota
	// (realizowane przez klase konkretna):
	// na poziomie MP jest to polecenie dla ECP.
	void execute_motion(void);

	// Zlecenie zakonczenia ruchu przez robota
	// (realizowane przez klase konkretna):
	// na poziomie MP jest to polecenie dla ECP.
	void terminate_ecp(void);

	void start_ecp(void);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_ROBOT_H_*/
