#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/common/ecp_robot.h"

class ecp_generator;

// klasa globalna dla calego procesu MP
class ecp_task : public ecp_mp_task
{
private:
    name_attach_t *ecp_attach, *trigger_attach; // by Y

    int MP_fd;
    // Wysyla puls do Mp przed oczekiwaniem na spotkanie
    void send_pulse_to_mp(int pulse_code, int pulse_value);

    // Receive of mp message
    int receive_mp_message(void);

    // Badanie typu polecenia z MP
    MP_COMMAND mp_command_type(void) const;

    // sprawdza czy przeszedl puls do ECP lub MP
    bool pulse_check();

protected:
    // Oczekiwanie na polecenie START od MP
    bool ecp_wait_for_start(void);

    // Oczekiwanie na STOP
    void ecp_wait_for_stop(void);

    // Oczekiwanie na nowy stan od MP
    bool get_next_state(void);

public: // TODO: following packages should be 'protected'
    // Odpowiedz ECP do MP, pola do ew. wypelnienia przez generatory
    ECP_REPLY_PACKAGE ecp_reply;

    // Polecenie od MP dla TASKa
    MP_COMMAND_PACKAGE mp_command;

public:
    ecp_robot* ecp_m_robot;

    // KONSTRUKTOR
    ecp_task(configurator &_config);

    // dla gcc: `'class Foo' has virtual functions but non-virtualdestructor` warning.
    virtual ~ecp_task();

    void initialize_communication(void);

    // obsluga sygnalu
    virtual void catch_signal_in_ecp_task(int sig);

    virtual void terminate();

    // --------------------------------------------------------------------------
    // Zlecenie ruchu dla EDP
    void Move(ecp_generator& the_generator);

    // methods for ECP template to redefine in concrete classes
    virtual void task_initialization(void);
    virtual void main_task_algorithm(void);

    // Informacja dla MP o zakonczeniu zadania uzytkownika
    void ecp_termination_notice(void);

public: // TODO: what follows should be private method

    // Oczekiwanie na polecenie od MP
    bool mp_buffer_receive_and_send(void);

    // Ustawienie typu odpowiedzi z ECP do MP
    void set_ecp_reply(ECP_REPLY ecp_r);

};

ecp_task* return_created_ecp_task (configurator &_config);


// klasa podzadania
class ecp_sub_task
{

protected:
    ecp_task &ecp_t;

public:
    ecp_sub_task(ecp_task &_ecp_t);
};

#endif /* _ECP_TASK_H */
