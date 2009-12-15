#ifndef MP_ROBOT_H_
#define MP_ROBOT_H_

#include "mp/task/mp_task.h"
#include "ecp_mp/ecp_mp_robot.h"

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
    //! deskryptor wezla na ktorym jest powolane ECP oraz jego PID
    uint32_t nd;
    pid_t ECP_pid;

protected:


    task::task &mp_object;

public:
    lib::MP_COMMAND_PACKAGE mp_command;      // Bufor z rozkazem dla ECP
    lib::ECP_REPLY_PACKAGE ecp_reply_package;        // Bufor z odpowiedzia z ECP

    bool communicate; // okresla czy robot ma byc obslugiwany w Move

    lib::sr_ecp &sr_ecp_msg;    // obiekt do komunikacji z SR

#if !defined(USE_MESSIP_SRR)
    //! main ECP request channel
    int ECP_fd;
#else
    //! main ECP request channel
    messip_channel_t* ECP_fd;
#endif
    //! A server connection ID identifing UI
    int scoid;

    //! flag indicating opened pulse connection from UI
    bool opened;

    char pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
    bool new_pulse; // okresla czy jest nowy puls
    bool new_pulse_checked; // okresla czy czy nowy puls zostal juz uwzgledniony w generatorze

      robot (lib::robot_name_t l_robot_name, const char* _section_name, task::task &mp_object_l);
    virtual ~robot();

    class MP_error
    {  // Klasa obslugi bledow robotow
    public:
        const lib::error_class_t error_class;
        const uint64_t error_no;
        MP_error (lib::error_class_t err0, uint64_t err1);
    };

    // Zlecenie wykonania ruchu przez robota
    // (realizowane przez klase konkretna):
    // na poziomie MP jest to polecenie dla ECP.
    void execute_motion (void);

    // Zlecenie zakonczenia ruchu przez robota
    // (realizowane przez klase konkretna):
    // na poziomie MP jest to polecenie dla ECP.
    void terminate_ecp (void);

    void start_ecp ( void );



};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_ROBOT_H_*/
