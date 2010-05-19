#ifndef MP_ROBOT_H_
#define MP_ROBOT_H_

#include "mp/task/mp_task.h"
#include "ecp_mp/ecp_mp_robot.h"

#include <time.h>

namespace mrrocpp {
namespace mp {
namespace robot {

class robot : public ecp_mp::robot
{
	// Klasa bazowa dla robotow (klasa abstrakcyjna)
	// Kazdy robot konkretny (wyprowadzony z klasy bazowej)
	// musi zawierac pola danych (skladowe) dotyczace
	// ostatnio zrealizowanej pozycji oraz pozycji zadanej
private:
#if !defined(PROCESS_SPAWN_RSH)
	//! deskryptor wezla na ktorym jest powolane ECP oraz jego PID
	uint32_t nd;
#endif

	pid_t ECP_pid;

#if !defined(USE_MESSIP_SRR)
	//! main ECP request channel
	int ECP_fd;
#else
	//! main ECP request channel
	messip_channel_t* ECP_fd;
#endif

protected:

	task::task &mp_object;

public:

	//ew. koordynacja ciagla domyslnie wylaczona ma wplyw na instrukcje move
	bool continuous_coordination;

	// Wysyla puls do Mp przed oczekiwaniem na spotkanie
	void send_pulse_to_ecp(int pulse_code, int pulse_value = 1);

	lib::MP_COMMAND_PACKAGE mp_command; // Bufor z rozkazem dla ECP
	lib::ECP_REPLY_PACKAGE ecp_reply_package; // Bufor z odpowiedzia z ECP

	struct timespec pulse_receive_time;

	bool communicate; // okresla czy robot ma byc obslugiwany w Move

	lib::sr_ecp &sr_ecp_msg; // obiekt do komunikacji z SR

	//! A server connection ID identifying UI
	int scoid;

	//! flag indicating opened pulse connection from UI
	bool opened;

	char pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
	bool new_pulse; // okresla czy jest nowy puls
	bool new_pulse_checked; // okresla czy czy nowy puls zostal juz uwzgledniony w generatorze

	robot(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l);
	virtual ~robot();

	class MP_error
	{ // Klasa obslugi bledow robotow
	public:
		const lib::error_class_t error_class;
		const uint64_t error_no;
		MP_error(lib::error_class_t err0, uint64_t err1);
	};

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
