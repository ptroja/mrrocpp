#ifndef MP_ROBOT_H_
#define MP_ROBOT_H_

#include "mp/mp_task.h"
#include "ecp_mp/ecp_mp_robot.h"


// ------------------------------------------------------------------------
struct robot_ECP_transmission_data : robot_transmission_data
{
public:
    MP_COMMAND mp_command;                // polecenie przesylane z MP do ECP
    ECP_REPLY  ecp_reply;                 // odpowiedz z ECP do MP

    ecp_next_state_t ecp_next_state;

    // speech command interface
    char commandRecognized[SPEECH_RECOGNITION_TEXT_LEN];

};
// ------------------------------------------------------------------------




class mp_robot : public ecp_mp_robot
{
    // Klasa bazowa dla robotow (klasa abstrakcyjna)
    // Kazdy robot konkretny (wyprowadzony z klasy bazowej)
    // musi zawierac pola danych (skladowe) dotyczace
    // ostatnio zrealizowanej pozycji oraz pozycji zadanej
protected:
    MP_COMMAND_PACKAGE mp_command;      // Bufor z rozkazem dla ECP
    // - uzytkownik nie powinien z tego korzystac
    ECP_REPLY_PACKAGE ecp_reply_package;        // Bufor z odpowiedzia z ECP
    // - uzytkownik nie powinien z tego korzystac

    mp_task &mp_object;

public:
    bool communicate; // okresla czy robot ma byc obslugiwany w Move

    sr_ecp &sr_ecp_msg;    // obiekt do komunikacji z SR

    //! deskryptor wezla na ktorym jest powolane ECP oraz jego PID (dla mp_task::kill_all_ECP)
    uint32_t nd;
    pid_t ECP_pid;

#if !defined(USE_MESSIP_SRR)
    int ECP_fd;	// deskryptor do komunikacji z ECP
#else
    messip_channel_t *ECP_fd;
#endif

    int32_t scoid; // server connection id
    char pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
    bool new_pulse; // okresla czy jest nowy puls
    bool robot_new_pulse_checked; // okresla czy czy nowy puls zostal juz uwzgledniony w generatorze

    robot_ECP_transmission_data ecp_td; // Obraz danych robota wykorzystywanych przez generator
    // - do uzytku uzytkownika (generatora)

    mp_robot (ROBOT_ENUM l_robot_name, const char* _section_name, mp_task &mp_object_l);
    ~mp_robot();

    class MP_error
    {  // Klasa obslugi bledow robotow
    public:
        const ERROR_CLASS error_class;
        const uint64_t mp_error;
        MP_error (ERROR_CLASS err0, uint64_t err1);
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

    // wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w swych skladowych
    // Ten bufor znajduje sie w robocie
    void create_command ( void );

    virtual void create_next_pose_command (void)
    {}

    // pobiera z pakietu przeslanego z EDP informacje i wstawia je do odpowiednich swoich skladowych
    virtual void get_reply ( void );
};

#endif /*MP_ROBOT_H_*/
