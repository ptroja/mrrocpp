#if !defined(_ECP_ROBOT_H)
#define _ECP_ROBOT_H

#include "common/com_buf.h"
#include "lib/srlib.h"
#include "lib/configurator.h"
#include "ecp_mp/ecp_mp_robot.h"

#include "messip/messip.h"

// ------------------------------------------------------------------------
class ecp_buffer: public ecp_command_buffer, public edp_reply_buffer
{
public:
    sr_ecp* sr_ecp_msg;     // by Y - Wskaznik na obiekt do komunikacji z SR
#if !defined(USE_MESSIP_SRR)

    void send  (int);
    void query (int);
#else

    void send  (messip_channel_t *);
    void query (messip_channel_t *);
#endif
};
// ------------------------------------------------------------------------


struct robot_EDP_transmission_data : robot_transmission_data
{

};

class ecp_task;

class ecp_robot : public ecp_mp_robot
{

    // Klasa bazowa dla robotow (klasa abstrakcyjna)
    // Kazdy robot konkretny (wyprowadzony z klasy bazowej)
    // musi zawierac pola danych (skladowe) dotyczace
    // ostatnio zrealizowanej pozycji oraz pozycji zadanej

    // int fd;	// by Y&W
protected:

    // int UI_fd; // by Y&W - przeniesione z procesu master
    pid_t EDP_MASTER_Pid; // Identyfikator procesu driver'a edp_m

    // strukture EDP_command_and_reply_buffer.instruction, ktora jest
    // nastepnie wyslana przez funkcje execute_motion() do EDP.
    // Struktura EDP_command_and_reply_buffer.reply_package zawierajaca
    // odpowiedz EDP na wyslany rozkaz, ktora moze byc wykorzystana
    // przez generator.next_step()
    // Funkcja generator.next_step() przygotowuje rozkazy dla EDP wypelniajac

    // by Y&W - przerzucenie zainicjowania komunikacji z procesu master do klasy
    void connect_to_edp (const char* edp_net_attach_point);
    
    ecp_buffer EDP_command_and_reply_buffer;

public:
    bool synchronised; // Flaga synchronizacji robota (true - zsynchronizowany, false - nie)

    int number_of_servos;

#if !defined(USE_MESSIP_SRR)

    int EDP_fd;	// by Y&W
#else

    messip_channel_t *EDP_fd;
#endif

     robot_EDP_transmission_data EDP_data; // Obraz robota wykorzystywany przez generator

    virtual void execute_motion (void);
    // Zlecenie wykonania ruchu przez robota (realizowane przez klase konkretna):
    // na poziomie ECP jest to polecenie dla EDP

    ecp_robot(ROBOT_ENUM _robot_name, configurator &_config, sr_ecp *_sr_ecp);
    ecp_robot(ROBOT_ENUM _robot_name, ecp_task& _ecp_object);

    pid_t get_EDP_pid(void) const;

    // destruktor by Y - do usuniecia obiektu do komunikacji z SR
    virtual ~ecp_robot(void);

    virtual void synchronise ( void );
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

    // Kopiowanie bufora przesylanego z MP do bufora wysylanego do EDP
    virtual void copy_mp_to_edp_buffer (c_buffer& mp_buffer);

    // by Y - o dziwo tego nie bylo !!!
    // Kopiowanie bufora przesylanego z EDP do bufora wysylanego do MP
    virtual void copy_edp_to_mp_buffer (r_buffer& mp_buffer);

    bool is_synchronised ( void ) const; // Czy robot zsynchronizowany?

    class ECP_error
    {  // Klasa obslugi bledow robota
    public:
        uint64_t error_class;
        uint64_t error_no;
        edp_error error;

        ECP_error ( uint64_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
    };

    class ECP_main_error
    {  // Klasa obslugi bledow ECP
    public:
        uint64_t error_class;
        uint64_t error_no;

        ECP_main_error ( uint64_t err_cl, uint64_t err_no);
    };

};

#endif /* _ECP_ROBOT_H */
