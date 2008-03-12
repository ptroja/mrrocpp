#include <iostream>
#include <map>



#include "mp/mp_generator.h"
#include "mp/mp_robot.h"



mp_generator::~mp_generator(void)
{ }
;

void mp_generator::re_run(void) // powrot do stanu wyjsciowego
{
    phase = BEFORE_FIRST_STEP;
    new_pulse_checked = true;
}

// kopiuje dane z robotow do generatora
void mp_generator::copy_data(std::map <ROBOT_ENUM, mp_robot*>& _robot_m)
{
    for (std::map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = _robot_m.begin();
            robot_m_iterator != _robot_m.end(); robot_m_iterator++)
    {
        robot_m_iterator->second->get_reply(); // odpowiedz z ECP
    }
}

// kopiuje polecenie stworzone w generatorze do robotow
void mp_generator::copy_generator_command (std::map <ROBOT_ENUM, mp_robot*>& _robot_m)
{
    for (std::map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = _robot_m.begin();
            robot_m_iterator != _robot_m.end(); robot_m_iterator++)
    {
        robot_m_iterator->second->create_command(); // rozkaz dla ECP
    }
}

mp_generator::mp_generator(mp_task& _mp_task)
        : ecp_mp_generator (*_mp_task.sr_ecp_msg),
        mp_t(_mp_task),
        wait_for_ECP_pulse(false),
        phase(BEFORE_FIRST_STEP),
        new_pulse_checked(true)
{}




// ---------------------------------------------------------------
bool mp_generator::Move ()
{
    // Funkcja zwraca false gdy samoistny koniec ruchu
    // Funkcja zwraca true gdy koniec ruchu wywolany jest przez STOP

    // czyszczenie aby nie czekac na pulsy z ECP
    for (std::map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = mp_t.robot_m.begin();
            robot_m_iterator != mp_t.robot_m.end(); robot_m_iterator++)
    {
        if (robot_m_iterator->second->new_pulse)
        {
            robot_m_iterator->second->robot_new_pulse_checked = false;
        }
    }

    // by Y - linia ponizej dodana 26.02.2007 - usunac komentarz jak bedzie dzialalo
    // ze wzgledu na obluge pulsow z UI w szczegolnosci stopu i wstrzymania
    if (mp_t.mp_receive_ui_or_ecp_pulse (mp_t.robot_m, *this))
        return true;

    // czyszczenie aby nie czekac na pulsy z ECP
    for (std::map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = mp_t.robot_m.begin();
            robot_m_iterator != mp_t.robot_m.end(); robot_m_iterator++)
    {
        if (robot_m_iterator->second->new_pulse)
        {
            robot_m_iterator->second->robot_new_pulse_checked = false;
        }
    }
    node_counter = 0;
    // (Inicjacja) generacja pierwszego kroku ruchu
    if (!first_step() )
        return false;

    do
    { // realizacja ruchu

        // zadanie przygotowania danych od czujnikow
        mp_t.all_sensors_initiate_reading(sensor_m);


        copy_generator_command( robot_m );

        // wykonanie kroku ruchu przez wybrane roboty (z flaga 'communicate')
        mp_t.execute_all(robot_m);

        copy_data( robot_m );

        // odczytanie danych z wszystkich czujnikow
        mp_t.all_sensors_get_reading(sensor_m);

        // oczekiwanie na puls z ECP lub UI
        if (mp_t.mp_receive_ui_or_ecp_pulse(mp_t.robot_m, *this))
            return true;
        node_counter++;
    }
    while ( next_step() );

    return false;
}
// ------------------------------------------------------------------------

