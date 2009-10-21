#if !defined(_ECP_ROBOT_H)
#define _ECP_ROBOT_H

#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/configurator.h"
#include "ecp_mp/ecp_mp_robot.h"

#include "lib/messip/messip.h"
class ui_common_robot;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class transparent;

}

namespace task {
class task;
} // namespace task

class ecp_robot : public ecp_mp::robot
{
	friend class ui_common_robot;
	friend class ecp::common::generator::transparent;

	// Klasa bazowa dla robotow (klasa abstrakcyjna)
	// Kazdy robot konkretny (wyprowadzony z klasy bazowej)
	// musi zawierac pola danych (skladowe) dotyczace
	// ostatnio zrealizowanej pozycji oraz pozycji zadanej
private:
	// Kopiowanie bufora przesylanego z MP do bufora wysylanego do EDP
	void copy_mp_to_edp_buffer (lib::c_buffer& mp_buffer);

	// by Y - o dziwo tego nie bylo !!!
	// Kopiowanie bufora przesylanego z EDP do bufora wysylanego do MP
	void copy_edp_to_mp_buffer (lib::r_buffer& mp_buffer);

	// zainicjowanie komunikacji
	void connect_to_edp (lib::configurator &config);

	pid_t EDP_MASTER_Pid; // Identyfikator procesu driver'a edp_m

	const bool spawn_and_kill;

protected:

	// strukture ecp_command.instruction, ktora jest
	// nastepnie wyslana przez funkcje execute_motion() do EDP.
	// Struktura reply_package zawierajaca
	// odpowiedz EDP na wyslany rozkaz, ktora moze byc wykorzystana
	// przez generator.next_step()
	// Funkcja generator.next_step() przygotowuje rozkazy dla EDP wypelniajac

public:

	lib::ecp_command_buffer ecp_command;
	lib::r_buffer reply_package;

	lib::sr_ecp & sr_ecp_msg;     // obiekt do komunikacji z SR

	bool synchronised; // Flaga synchronizacji robota (true - zsynchronizowany, false - nie)

	void send  ();
	void query ();

	int number_of_servos;

#if !defined(USE_MESSIP_SRR)
	int EDP_fd;	// by Y&W
#else
	messip_channel_t *EDP_fd;
#endif

	ecp_mp::robot_transmission_data EDP_data; // Obraz robota wykorzystywany przez generator

	virtual void execute_motion (void);
	// Zlecenie wykonania ruchu przez robota (realizowane przez klase konkretna):
	// na poziomie ECP jest to polecenie dla EDP

	ecp_robot(lib::robot_name_t _robot_name, lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	ecp_robot(lib::robot_name_t _robot_name, common::task::task& _ecp_object);

	pid_t get_EDP_pid(void) const;

	// destruktor by Y - do usuniecia obiektu do komunikacji z SR
	virtual ~ecp_robot(void);

	void synchronise(void);
	// Zlecenie synchronizacji robota
	// Pobranie aktualnych polozen

	virtual void create_command (void) = 0;
	// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w obrazie
	// robota wykorzystywanych przez generator
	// Ten bufor znajduje sie w robocie

	virtual void get_reply (void)  = 0;
	// pobiera z pakietu przeslanego z EDP informacje i wstawia je do
	// odpowiednich skladowych obrazu robota wykorzystywanych przez generator
	// Ten bufor znajduje sie w robocie

	bool is_synchronised ( void ) const; // Czy robot zsynchronizowany?

	class ECP_error
	{  // Klasa obslugi bledow robota
	public:
		uint64_t error_class;
		uint64_t error_no;
		lib::edp_error error;

		ECP_error ( uint64_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
	};

	class ECP_main_error
	{  // Klasa obslugi bledow ECP
	public:
		const lib::ERROR_CLASS error_class;
		const uint64_t error_no;

		ECP_main_error ( lib::ERROR_CLASS err_cl, uint64_t err_no);
	};
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_ROBOT_H */
