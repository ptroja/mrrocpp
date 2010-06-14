#include <string.h>
#include <math.h>
#include <errno.h>
#include <ctype.h>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include "ecp/common/generator/ecp_g_teach_in.h"

#include "lib/exception.h"
#include <boost/throw_exception.hpp>
#include <boost/exception/errinfo_errno.hpp>
#include <boost/exception/errinfo_api_function.hpp>

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// ####################################################################################################
// ############################     Odtwarzanie listy pozycji    ######################################
// ####################################################################################################

// ####################################################################################################
// ecp_g_teach_in - klasa bazowa
// ####################################################################################################

teach_in::teach_in(common::task::task& _ecp_task) :
	generator(_ecp_task)
  {
    pose_list.clear();
    pose_list_iterator = pose_list.end();
  }

// -------------------------------------------------------
// destruktor
teach_in::~teach_in(void)
  {
    flush_pose_list();
  }

// --------------------------------------------------------------------------
void teach_in::teach(lib::ECP_POSE_SPECIFICATION ps, const char *msg)
  { // Uczenie robota
    lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
    lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
    bool first_time = true; // Znacznik

    // by Y - okreslenie robota

    ecp_to_ui_msg.robot_name = the_robot->robot_name;

    ecp_to_ui_msg.ecp_message = convert(ps); // Rodzaj wspolrzednych, w ktorych uczony jest robot
    strncpy(ecp_to_ui_msg.string, msg, MSG_LENGTH); // Komunikat przesylany do UI podczas uczenia
    for (;;)
      {
        // Polecenie uczenia do UI
#if !defined(USE_MESSIP_SRR)
    	ecp_to_ui_msg.hdr.type=0;
    	if (MsgSend(ecp_t.UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0)
#else
    	if(messip::port_send(ecp_t.UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0)
#endif
          {
    		BOOST_THROW_EXCEPTION(
    			lib::exception::System_error() <<
    			boost::errinfo_errno(errno)
    		);
          }
        if (ui_to_ecp_rep.reply == lib::QUIT) // Koniec uczenia
          break;
        if (first_time)
          {
            // Tworzymy glowe listy
            first_time = false;
            create_pose_list_head(ps, ui_to_ecp_rep.double_number,
                ui_to_ecp_rep.coordinates);
          }
        else
          {
            // Wstaw do listy nowa pozycje
            insert_pose_list_element(ps, ui_to_ecp_rep.double_number,
                ui_to_ecp_rep.coordinates);
          }
      }
    initiate_pose_list();
  } // end: teach()


// --------------------------------------------------------------------------
// Zapis trajektorii do pliku
void teach_in::save_file(lib::ECP_POSE_SPECIFICATION ps)
  {
    lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
    lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
    ecp_taught_in_pose tip; // Zapisywana pozycja
    char *cwd; // Wsk. na nazwe biezacego katalogu
    char coordinate_type[80]; // Opis wspolrzednych: "MOTOR", "JOINT", ...
    uint64_t e; // Kod bledu systemowego
    uint64_t number_of_poses; // Liczba pozycji do zapamietania
    uint64_t i, j; // Liczniki petli

    ecp_to_ui_msg.ecp_message = lib::SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
    strcpy(ecp_to_ui_msg.string, "*.trj"); // Wzorzec nazwy pliku
    // if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
#if !defined(USE_MESSIP_SRR)
    ecp_to_ui_msg.hdr.type=0;
    if (MsgSend(ecp_t.UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0)
#else
    if(messip::port_send(ecp_t.UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0)
#endif
    {
		BOOST_THROW_EXCEPTION(
			lib::exception::System_error() <<
			boost::errinfo_errno(errno)
		);
      }
    if (ui_to_ecp_rep.reply == lib::QUIT)
      { // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
        return;
      }

    switch (ps)
      {
    case lib::ECP_MOTOR:
      strcpy(coordinate_type, "MOTOR");
      break;
    case lib::ECP_JOINT:
      strcpy(coordinate_type, "JOINT");
      break;
    case lib::ECP_PF_VELOCITY:
      strcpy(coordinate_type, "POSE_FORCE_TORQUE_AT_FRAME");
      break;

    default:
      strcpy(coordinate_type, "MOTOR");
      }
    cwd = getcwd(NULL, 0);
    if (chdir(ui_to_ecp_rep.path) != 0)
      {
        perror(ui_to_ecp_rep.path);
        BOOST_THROW_EXCEPTION(
        	lib::exception::NonFatal_error() <<
        	lib::exception::error_code(NON_EXISTENT_DIRECTORY) <<
        	boost::errinfo_errno(errno) <<
        	boost::errinfo_api_function("chdir")
        );
      }
    std::ofstream to_file(ui_to_ecp_rep.filename); // otworz plik do zapisu
    e = errno;
    if (!to_file)
      {
        perror(ui_to_ecp_rep.filename);
        BOOST_THROW_EXCEPTION(
        	lib::exception::Fatal_error() <<
        	lib::exception::error_code(NON_EXISTENT_FILE) <<
        	boost::errinfo_errno(errno) <<
        	boost::errinfo_api_function("chdir")
        );
      }
    else
      {
        initiate_pose_list();
        number_of_poses = pose_list_length();
        to_file << coordinate_type << '\n';
        to_file << number_of_poses << '\n';
        for (i = 0; i < number_of_poses; i++)
          {
            get_pose(tip);
            to_file << tip.motion_time << ' ';
            for (j = 0; j < MAX_SERVOS_NR; j++)
              to_file << tip.coordinates[j] << ' ';
            if (ps == lib::ECP_PF_VELOCITY)
              { // by Y
                to_file << tip.extra_info << ' ';
              }
            to_file << '\n';
            next_pose_list_ptr();
          }
        initiate_pose_list();
      }
  } // end: irp6_on_track_save_file()
  // --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Wczytanie trajektorii z pliku
bool teach_in::load_file_from_ui()
  {
    // Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,
    // false w przeciwnym razie
    lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
    lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP

    ecp_to_ui_msg.ecp_message = lib::LOAD_FILE; // Polecenie wprowadzenia nazwy odczytywanego pliku

#if !defined(USE_MESSIP_SRR)
    ecp_to_ui_msg.hdr.type=0;
    if (MsgSend(ecp_t.UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0)
#else
    if(messip::port_send(ecp_t.UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0)
#endif
    {
		BOOST_THROW_EXCEPTION(
			lib::exception::System_error() <<
			boost::errinfo_errno(errno)
		);
      }
    if (ui_to_ecp_rep.reply == lib::QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
      return false;
    if (chdir(ui_to_ecp_rep.path) != 0)
      {
        perror(ui_to_ecp_rep.path);
        BOOST_THROW_EXCEPTION(
        	lib::exception::NonFatal_error() <<
        	lib::exception::error_code(NON_EXISTENT_DIRECTORY) <<
        	boost::errinfo_errno(errno) <<
        	boost::errinfo_api_function("chdir")
        );
      }

    return load_file_with_path(ui_to_ecp_rep.filename);

  } // end: load_file()
  // --------------------------------------------------------------------------


bool teach_in::load_file_with_path(const std::string & file_name)
  {
    // Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,

    char coordinate_type[80]; // Opis wspolrzednych: "MOTOR", "JOINT", ...
    lib::ECP_POSE_SPECIFICATION ps; // Rodzaj wspolrzednych

    uint64_t number_of_poses; // Liczba zapamietanych pozycji
    uint64_t i, j; // Liczniki petli
    bool first_time = true; // Znacznik
    double coordinates[MAX_SERVOS_NR]; // Wczytane wspolrzedne
    int extra_info = 0; // by Y - dodatkowe info do dowolnego wykorzystania
    double motion_time; // Czas dojscia do wspolrzednych


    std::ifstream from_file(file_name.c_str()); // otworz plik do odczytu
    if (!from_file.good())
      {
        perror(file_name.c_str());
        BOOST_THROW_EXCEPTION(
        	lib::exception::NonFatal_error() <<
        	lib::exception::error_code(NON_EXISTENT_FILE) <<
        	boost::errinfo_errno(errno)
        );
      }

    if ( !(from_file >> coordinate_type))
      {
        BOOST_THROW_EXCEPTION(
        	lib::exception::NonFatal_error() <<
        	lib::exception::error_code(READ_FILE_ERROR) <<
        	boost::errinfo_errno(errno)
        );
      }

    // Usuwanie spacji i tabulacji
    i = 0;
    j = 0;
    while (coordinate_type[i] == ' ' || coordinate_type[i] == '\t')
      i++;
    while (coordinate_type[i] != ' ' && coordinate_type[i] != '\t'
        && coordinate_type[i] != '\n' && coordinate_type[i] != '\r'
        && coordinate_type[j] != '\0')
      {
        coordinate_type[j] = toupper(coordinate_type[i]);
        i++;
        j++;
      }
    coordinate_type[j] = '\0';

    if ( !strcmp(coordinate_type, "MOTOR") )
      {
    //	fprintf(stderr, "STRANGE_GET_ARM_REQUEST@%s:%d\n", __FILE__, __LINE__);
        ps = lib::ECP_MOTOR;
      }
    else if ( !strcmp(coordinate_type, "JOINT") )
      ps = lib::ECP_JOINT;
     else if ( !strcmp(coordinate_type, "lib::PF_VELOCITY") )
      ps = lib::ECP_PF_VELOCITY;
    else
      {
        BOOST_THROW_EXCEPTION(
        	lib::exception::NonFatal_error() <<
        	lib::exception::error_code(NON_TRAJECTORY_FILE) <<
        	boost::errinfo_errno(errno)
        );
      }
    if ( !(from_file >> number_of_poses))
      {
        BOOST_THROW_EXCEPTION(
        	lib::exception::NonFatal_error() <<
        	lib::exception::error_code(READ_FILE_ERROR) <<
        	boost::errinfo_errno(errno)
        );
      }
    flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
    for (i = 0; i < number_of_poses; i++)
      {
        if (!(from_file >> motion_time))
          {
            BOOST_THROW_EXCEPTION(
            	lib::exception::NonFatal_error() <<
            	lib::exception::error_code(READ_FILE_ERROR) <<
            	boost::errinfo_errno(errno)
            );
          }
        for (j = 0; j < MAX_SERVOS_NR; j++)
          {
            if ( !(from_file >> coordinates[j]))
              { // Zabezpieczenie przed danymi nienumerycznymi
                BOOST_THROW_EXCEPTION(
                	lib::exception::NonFatal_error() <<
                	lib::exception::error_code(READ_FILE_ERROR) <<
                	boost::errinfo_errno(errno)
                );
              }
          }

        if (ps == lib::ECP_PF_VELOCITY)
          { // by Y
            if ( !(from_file >> extra_info))
              { // Zabezpieczenie przed danymi nienumerycznymi
                BOOST_THROW_EXCEPTION(
                	lib::exception::NonFatal_error() <<
                	lib::exception::error_code(READ_FILE_ERROR) <<
                	boost::errinfo_errno(errno)
                );
              }
            if (first_time)
              {
                // Tworzymy glowe listy
                first_time = false;
                create_pose_list_head(ps, motion_time, coordinates, extra_info);
              }
            else
              {
                // Wstaw do listy nowa pozycje
                insert_pose_list_element(ps, motion_time, coordinates, extra_info);
              }
          }
        else
          {
            if (first_time)
              {
                // Tworzymy glowe listy
                first_time = false;
                create_pose_list_head(ps, motion_time, coordinates);
              }
            else
              {
                // Wstaw do listy nowa pozycje
                insert_pose_list_element(ps, motion_time, coordinates);
              }
          }
      } // end: for

    return true;
  } // end: load_file()


// -------------------------------------------------------
void teach_in::flush_pose_list(void)
  {
    pose_list.clear();
  } // end: flush_pose_list
  // -------------------------------------------------------
void teach_in::initiate_pose_list(void)
  {
    pose_list_iterator = pose_list.begin();
  }
// -------------------------------------------------------
void teach_in::next_pose_list_ptr(void)
  {
    if (pose_list_iterator != pose_list.end())
      pose_list_iterator++;
  }
// -------------------------------------------------------
void teach_in::get_pose(ecp_taught_in_pose& tip)
  { // by Y
    tip = *pose_list_iterator;
  }
// -------------------------------------------------------
// Pobierz nastepna pozycje z listy
void teach_in::get_next_pose(double next_pose[MAX_SERVOS_NR])
  {
    memcpy(next_pose, pose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
  }
// -------------------------------------------------------
void teach_in::set_pose(lib::ECP_POSE_SPECIFICATION ps,
    double motion_time, double coordinates[MAX_SERVOS_NR], int extra_info)
  {
    pose_list_iterator->arm_type = ps;
    pose_list_iterator->motion_time = motion_time;
    pose_list_iterator->extra_info = extra_info;
    memcpy(pose_list_iterator->coordinates, coordinates, MAX_SERVOS_NR*sizeof(double));
  }
// -------------------------------------------------------
bool teach_in::is_pose_list_element(void)
  {
    // sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
    if (pose_list_iterator != pose_list.end())
      return true;
    else
      return false;
  }
// -------------------------------------------------------
bool teach_in::is_last_list_element(void)
  {
    // sprawdza czy aktualnie wskazywany element listy ma nastepnik
    // jesli <> nulla
    if (pose_list_iterator != pose_list.end() )
      {
        if ( (++pose_list_iterator) != pose_list.end() )
          {
            --pose_list_iterator;
            return false;
          }
        else
          {
            --pose_list_iterator;
            return true;
          }; // end if
      }
    return false;
  }
// -------------------------------------------------------

void teach_in::create_pose_list_head(lib::ECP_POSE_SPECIFICATION ps,
    double motion_time, const double coordinates[MAX_SERVOS_NR], int extra_info)
{
	pose_list.push_back(ecp_taught_in_pose(ps, motion_time, coordinates, extra_info));
	pose_list_iterator = pose_list.begin();
}

void teach_in::insert_pose_list_element(lib::ECP_POSE_SPECIFICATION ps,
    double motion_time, const double coordinates[MAX_SERVOS_NR], int extra_info)
  {
    pose_list.push_back(ecp_taught_in_pose(ps, motion_time,
        coordinates, extra_info));
    pose_list_iterator++;
  }

// -------------------------------------------------------
int teach_in::pose_list_length(void)
  {
    return pose_list.size();
  }

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda    first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool teach_in::first_step()
  {
    //	 printf("w irp6ot_teach_in_generator::first_step\n");
 //printf(stderr, "DEBUG@%s:%d\n", __FILE__, __LINE__);
    initiate_pose_list();
    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM

    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
    the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
    the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = 8;
    the_robot->ecp_command.instruction.value_in_step_no = 6;

    //	 printf("w irp6ot_teach_in_generator::first_step za initiate_pose_list\n");
    // return next_step ();
    return true;
  }

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda    next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool teach_in::next_step()
  {
    ecp_taught_in_pose tip; // Nauczona pozycja

    // printf("W irp6ot_teach_in_generator::next_step\n");
    if (is_pose_list_element())
      {
      }
    else
      {

        return false;
      }

    get_pose(tip);
    // Przepisanie pozycji z listy
    switch (tip.arm_type)
      {
    case lib::ECP_MOTOR:
  //  	fprintf(stderr, "DEBUG@%s:%d\n", __FILE__, __LINE__);
      the_robot->ecp_command.instruction.instruction_type = lib::SET;
      the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
      the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
      the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
      the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
      the_robot->ecp_command.instruction.motion_steps = (uint16_t) ceil(tip.motion_time/STEP);
      the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps;
      memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates,
      MAX_SERVOS_NR*sizeof (double));
      break;
    case lib::ECP_JOINT:
  //   	fprintf(stderr, "DEBUG@%s:%d\n", __FILE__, __LINE__);
      the_robot->ecp_command.instruction.instruction_type = lib::SET;
      the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
      the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
      the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
      the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
      the_robot->ecp_command.instruction.motion_steps = (uint16_t) ceil(tip.motion_time/STEP);
      the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps;
      memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates,
      MAX_SERVOS_NR*sizeof (double));
      // printf("lumpu: %f\n", the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[6]);
      break;
    default:
        BOOST_THROW_EXCEPTION(
        	lib::exception::NonFatal_error() <<
        	lib::exception::error_code(INVALID_POSE_SPECIFICATION) <<
        	boost::errinfo_errno(errno)
        );
      } // end: switch

    next_pose_list_ptr(); // nastepna pozycja
    return true;

  }

lib::ECP_TO_UI_COMMAND teach_in::convert(lib::ECP_POSE_SPECIFICATION ps) const
  {
    switch (ps)
      {
    case lib::ECP_MOTOR:
      return lib::C_MOTOR;
    case lib::ECP_JOINT:
      return lib::C_JOINT;
    default:
      return lib::C_MOTOR;
      }
  }

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
