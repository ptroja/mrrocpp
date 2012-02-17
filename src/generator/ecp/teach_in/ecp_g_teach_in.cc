/*!
 * @file
 * @brief File contains teach_in generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstring>
#include <cmath>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <unistd.h>

#include "base/ecp/ecp_exceptions.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"

#include "ecp_g_teach_in.h"

#include "base/lib/messip/messip_dataport.h"

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
		common::generator::generator(_ecp_task)
{
	generator_name = ecp_mp::generator::ECP_GEN_TEACH_IN;

	pose_list.clear();
	pose_list_iterator = pose_list.end();
}

// -------------------------------------------------------
// destruktor
teach_in::~teach_in(void)
{
}

// --------------------------------------------------------------------------
void teach_in::teach(lib::ECP_POSE_SPECIFICATION ps, const char *msg)
{ // Uczenie robota
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
	uint64_t e; // kod bledu systemowego
	bool first_time = true; // Znacznik

	// by Y - okreslenie robota

	ecp_to_ui_msg.robot_name = the_robot->robot_name;

	ecp_to_ui_msg.ecp_message = convert(ps); // Rodzaj wspolrzednych, w ktorych uczony jest robot
	strncpy(ecp_to_ui_msg.string, msg, MSG_LENGTH); // Komunikat przesylany do UI podczas uczenia
	for (;;) {
		// Polecenie uczenia do UI

		if (messip::port_send(ecp_t.UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0)

		{ // Y&W

			e = errno;
			perror("ecp teach(): Send() to UI failed");
			sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
			BOOST_THROW_EXCEPTION(exception::se_g());
		}
		if (ui_to_ecp_rep.reply == lib::QUIT) // Koniec uczenia
			break;
		if (first_time) {
			// Tworzymy glowe listy
			first_time = false;
			create_pose_list_head(ps, ui_to_ecp_rep.double_number, ui_to_ecp_rep.coordinates);
		} else {
			// Wstaw do listy nowa pozycje
			insert_pose_list_element(ps, ui_to_ecp_rep.double_number, ui_to_ecp_rep.coordinates);
		}
	} // end: for(;;)
	initiate_pose_list();
} // end: teach()

// --------------------------------------------------------------------------
// Zapis trajektorii do pliku
void teach_in::save_file(lib::ECP_POSE_SPECIFICATION ps)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
	ecp_taught_in_pose tip; // Zapisywana pozycja
	char coordinate_type[80]; // Opis wspolrzednych: "MOTOR", "JOINT", ...

	ecp_to_ui_msg.ecp_message = lib::SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.trj"); // Wzorzec nazwy pliku
	// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {

	if (messip::port_send(ecp_t.UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0)

	{ // by Y&W
		int e = errno;
		perror("ecp: Send() to UI failed");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(exception::se_g());
	}
	if (ui_to_ecp_rep.reply == lib::QUIT) { // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
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
			break;
	}

	if (chdir(ui_to_ecp_rep.path) != 0) {
		perror(ui_to_ecp_rep.path);
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_EXISTENT_DIRECTORY));
	}

	std::ofstream to_file(ui_to_ecp_rep.filename); // otworz plik do zapisu

	if (!to_file) {
		perror(ui_to_ecp_rep.filename);
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_EXISTENT_FILE));
	} else {
		initiate_pose_list();
		std::size_t number_of_poses = pose_list_length();
		to_file << coordinate_type << '\n';
		to_file << number_of_poses << '\n';
		for (int i = 0; i < number_of_poses; i++) {
			tip = get_pose();
			to_file << tip.motion_time << ' ';
			for (int j = 0; j < lib::MAX_SERVOS_NR; j++)
				to_file << tip.coordinates[j] << ' ';
			if (ps == lib::ECP_PF_VELOCITY) { // by Y
				to_file << tip.extra_info << ' ';
			}
			to_file << '\n';
			next_pose_list_ptr();
		}
		initiate_pose_list();
	}
}
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

	if (messip::port_send(ecp_t.UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) { // by Y&W
		int e = errno;
		perror("ecp: Send() to UI failed");
		sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(exception::se_g());
	}

	if (ui_to_ecp_rep.reply == lib::QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return false;

	if (chdir(ui_to_ecp_rep.path) != 0) {
		perror(ui_to_ecp_rep.path);
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_EXISTENT_DIRECTORY));
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
	double coordinates[lib::MAX_SERVOS_NR]; // Wczytane wspolrzedne
	int extra_info = 0; // by Y - dodatkowe info do dowolnego wykorzystania
	double motion_time; // Czas dojscia do wspolrzednych

	std::ifstream from_file(file_name.c_str()); // otworz plik do odczytu
	if (!from_file.good()) {
		perror(file_name.c_str());
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_EXISTENT_FILE));
	}

	if (!(from_file >> coordinate_type)) {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
	}

	// Usuwanie spacji i tabulacji
	i = 0;
	j = 0;
	while (coordinate_type[i] == ' ' || coordinate_type[i] == '\t')
		i++;
	while (coordinate_type[i] != ' ' && coordinate_type[i] != '\t' && coordinate_type[i] != '\n'
			&& coordinate_type[i] != '\r' && coordinate_type[j] != '\0') {
		coordinate_type[j] = toupper(coordinate_type[i]);
		i++;
		j++;
	}
	coordinate_type[j] = '\0';

	if (!strcmp(coordinate_type, "MOTOR")) {
		//	fprintf(stderr, "STRANGE_GET_ARM_REQUEST@%s:%d\n", __FILE__, __LINE__);
		ps = lib::ECP_MOTOR;
	} else if (!strcmp(coordinate_type, "JOINT"))
		ps = lib::ECP_JOINT;
	else if (!strcmp(coordinate_type, "lib::PF_VELOCITY"))
		ps = lib::ECP_PF_VELOCITY;
	else {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_TRAJECTORY_FILE));
	}
	if (!(from_file >> number_of_poses)) {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
	}
	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
	for (i = 0; i < number_of_poses; i++) {
		if (!(from_file >> motion_time)) {
			BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
		}
		for (j = 0; j < lib::MAX_SERVOS_NR; j++) {
			if (!(from_file >> coordinates[j])) { // Zabezpieczenie przed danymi nienumerycznymi
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
			}
		}

		if (ps == lib::ECP_PF_VELOCITY) { // by Y
			if (!(from_file >> extra_info)) { // Zabezpieczenie przed danymi nienumerycznymi
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
			}
			if (first_time) {
				// Tworzymy glowe listy
				first_time = false;
				create_pose_list_head(ps, motion_time, coordinates, extra_info);
			} else {
				// Wstaw do listy nowa pozycje
				insert_pose_list_element(ps, motion_time, coordinates, extra_info);
			}
		} else {
			if (first_time) {
				// Tworzymy glowe listy
				first_time = false;
				create_pose_list_head(ps, motion_time, coordinates);
			} else {
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
const ecp_taught_in_pose & teach_in::get_pose(void) const
{ // by Y
	return *pose_list_iterator;
}
// -------------------------------------------------------
// Pobierz nastepna pozycje z listy
void teach_in::get_next_pose(double next_pose[lib::MAX_SERVOS_NR])
{
	memcpy(next_pose, pose_list_iterator->coordinates, lib::MAX_SERVOS_NR * sizeof(double));
}
// -------------------------------------------------------
void teach_in::set_pose(lib::ECP_POSE_SPECIFICATION ps, double motion_time, double coordinates[lib::MAX_SERVOS_NR], int extra_info)
{
	pose_list_iterator->arm_type = ps;
	pose_list_iterator->motion_time = motion_time;
	pose_list_iterator->extra_info = extra_info;
	memcpy(pose_list_iterator->coordinates, coordinates, lib::MAX_SERVOS_NR * sizeof(double));
}
// -------------------------------------------------------
bool teach_in::is_pose_list_element(void) const
{
	// sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
	return (pose_list_iterator != pose_list.end());
}
// -------------------------------------------------------
bool teach_in::is_last_list_element(void) const
{
	// sprawdza czy aktualnie wskazywany element listy ma nastepnik
	// jesli <> nulla
	if (pose_list_iterator != pose_list.end()) {
		pose_list_t::const_iterator it = pose_list_iterator;
		it++;
		return (it == pose_list.end());
	}
	return false;
}
// -------------------------------------------------------

void teach_in::create_pose_list_head(lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info)
{
	pose_list.push_back(ecp_taught_in_pose(ps, motion_time, coordinates, extra_info));
	pose_list_iterator = pose_list.begin();
}

void teach_in::insert_pose_list_element(lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info)
{
	pose_list.push_back(ecp_taught_in_pose(ps, motion_time, coordinates, extra_info));
	pose_list_iterator++;
}

// -------------------------------------------------------
std::size_t teach_in::pose_list_length(void) const
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
	the_robot->ecp_command.get_type = ARM_DEFINITION; // ARM

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::MOTOR;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 8;
	the_robot->ecp_command.value_in_step_no = 6;

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
	if (is_pose_list_element()) {
	} else {

		return false;
	}

	tip = get_pose();
	// Przepisanie pozycji z listy
	switch (tip.arm_type)
	{
		case lib::ECP_MOTOR:
			//  	fprintf(stderr, "DEBUG@%s:%d\n", __FILE__, __LINE__);
			the_robot->ecp_command.instruction_type = lib::SET;
			the_robot->ecp_command.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.interpolation_type = lib::MIM;
			the_robot->ecp_command.motion_steps = (uint16_t) ceil(tip.motion_time / lib::EDP_STEP);
			the_robot->ecp_command.value_in_step_no = the_robot->ecp_command.motion_steps;
			memcpy(the_robot->ecp_command.arm.pf_def.arm_coordinates, tip.coordinates, lib::MAX_SERVOS_NR
					* sizeof(double));
			break;
		case lib::ECP_JOINT:
			//   	fprintf(stderr, "DEBUG@%s:%d\n", __FILE__, __LINE__);
			the_robot->ecp_command.instruction_type = lib::SET;
			the_robot->ecp_command.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.set_arm_type = lib::JOINT;
			the_robot->ecp_command.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.interpolation_type = lib::MIM;
			the_robot->ecp_command.motion_steps = (uint16_t) ceil(tip.motion_time / lib::EDP_STEP);
			the_robot->ecp_command.value_in_step_no = the_robot->ecp_command.motion_steps;
			memcpy(the_robot->ecp_command.arm.pf_def.arm_coordinates, tip.coordinates, lib::MAX_SERVOS_NR
					* sizeof(double));
			// printf("lumpu: %f\n", the_robot->ecp_command.arm.pf_def.arm_coordinates[6]);
			break;
		default:
			BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_POSE_SPECIFICATION));
	} // end: switch

	next_pose_list_ptr(); // nastepna pozycja
	return true;

}

lib::ECP_TO_UI_REQUEST teach_in::convert(lib::ECP_POSE_SPECIFICATION ps) const
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
