#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/common/ecp_robot.h"

class ecp_generator;

// klasa globalna dla calego procesu MP
class ecp_task: public ecp_mp_task  {
private:
	name_attach_t *ecp_attach, *trigger_attach;	// by Y

protected:
	pid_t MP_pid;

public:
	int MP_fd;
	MP_COMMAND_PACKAGE mp_command; // Polecenie od MP
	ECP_REPLY_PACKAGE ecp_reply;   // Odpowiedz ECP do MP
	
	ecp_robot* ecp_m_robot;
	
	// KONSTRUKTORY
	ecp_task();
	// dla gcc: `'class Foo' has virtual functions but non-virtualdestructor` warning.
	virtual ~ecp_task();
	
	// Przekazanie identyfikatora procesu MP
	void set_mp_pid ( pid_t mp_pid);
	
	// METODY
	// funkcja odbierajaca pulsy z ECP
	
	// obsluga sygnalu
	virtual void catch_signal_in_ecp_task(int sig);
	
	virtual void terminate();
	
	void initialize_communication ();
	
	// --------------------------------------------------------------------------
	// Zlecenie ruchu dla EDP
	void Move (ecp_generator& the_generator);
	
	// methods for ECP template to redefine in concrete classes
	virtual void task_initialization(void);
	virtual void main_task_algorithm(void);
	
	// Informacja dla MP o zakonczeniu zadania uzytkownika
	void ecp_termination_notice (void);
	
	// Oczekiwanie na polecenie START od MP
	bool ecp_wait_for_start (void);
	
	// Oczekiwanie na STOP
	void ecp_wait_for_stop (void);
	
	// Oczekiwanie na nowy stan od MP
	bool get_next_state (void);
	
	// Oczekiwanie na polecenie od MP
	bool mp_buffer_receive_and_send (void);
	
	// Receive of mp message
	int receive_mp_message (void);
	
	// Badanie typu polecenia z MP
	MP_COMMAND mp_command_type (void) const;
	
	// Ustawienie typu odpowiedzi z ECP do MP
	void set_ecp_reply ( ECP_REPLY ecp_r);
	
	// Wysyla puls do Mp przed oczekiwaniem na spotkanie  
	void send_pulse_to_mp (char pulse_code, long pulse_value);
	
	bool pulse_check(); // by Y - sprawdza czy przeszedl puls do ECP lub MP
	
	BYTE convert (POSE_SPECIFICATION ps);
};

ecp_task* return_created_ecp_task (void);

#endif /* _ECP_TASK_H */
