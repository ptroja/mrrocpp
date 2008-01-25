#include <string.h>

#include "ecp/common/ecp_operator_reaction_condition.h"

ecp_operator_reaction_condition::ecp_operator_reaction_condition (ecp_task& _ecp_task)
	:
		ecp_generator (_ecp_task, true)
{
	pose_list.clear();
	pose_list_iterator = pose_list.end();
	UI_fd = _ecp_task.UI_fd;
}

// destruktor
ecp_operator_reaction_condition::~ecp_operator_reaction_condition (void)
{
	flush_supplementary_list();
}

void ecp_operator_reaction_condition::flush_supplementary_list ( void )
{
	pose_list.clear();
}

void ecp_operator_reaction_condition::initiate_supplementary_list(void)
{
	pose_list_iterator = pose_list.begin();
}

void ecp_operator_reaction_condition::next_supplementary_list_ptr (void)
{
	if (pose_list_iterator != pose_list.end())
		pose_list_iterator++;
}

void ecp_operator_reaction_condition::get_supplementary (ecp_taught_in_pose& tip)
{
	tip.arm_type = pose_list_iterator->arm_type;
	tip.motion_time = pose_list_iterator->motion_time;
	memcpy(tip.coordinates, pose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
}

void ecp_operator_reaction_condition::set_supplementary (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR])
{
	pose_list_iterator->arm_type = ps;
	pose_list_iterator->motion_time = motion_time;
	memcpy(pose_list_iterator->coordinates, coordinates, MAX_SERVOS_NR*sizeof(double));
}

void ecp_operator_reaction_condition::create_supplementary_list_head (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR])
{
	pose_list.push_back(ecp_taught_in_pose(ps, motion_time, coordinates));
	pose_list_iterator = pose_list.begin();
}

void ecp_operator_reaction_condition::insert_supplementary_list_element (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR])
{
	pose_list.push_back(ecp_taught_in_pose(ps, motion_time, coordinates));
	pose_list_iterator++;
}

bool ecp_operator_reaction_condition::is_supplementary_list_element ( void )
{
	// sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
	if ( pose_list_iterator != pose_list.end()) {
		return true;
	} else {
		return false;
	}
}

bool ecp_operator_reaction_condition::is_supplementary_list_head ( void )
{
	// sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
	if ( pose_list_iterator == pose_list.begin() ) {
		return true;
	} else {
		return false;
	}
}

int ecp_operator_reaction_condition::supplementary_list_length(void)
{
	return pose_list.size();
}

// --------------------------------------------------------------------------
bool ecp_operator_reaction_condition::first_step ()
{
	// bada wartosc warunku poczatkowego
	// true - konczy czekanie (funkcja wait)
	// false - kontynuuje oczekiwanie
	// Przy spelnieniu warunku wczytuje stan robota w zaleznosci od rozkazu przygotowanego w EDP_data

	// Stworzenie rozkazu odczytu wspolrzednych kartezjanskich
	the_robot->communicate = false; // wylaczamy robota na jeden krok

	return true;

} // end: irp6ot_operator_reaction_condition::condition_value ()
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool ecp_operator_reaction_condition::next_step ()
{
	// bada wartosc warunku poczatkowego
	// true - konczy czekanie (funkcja wait)
	// false - kontynuuje oczekiwanie
	// Przy spelnieniu warunku wczytuje stan robota w zaleznosci od rozkazu przygotowanego w EDP_data

	// Stworzenie rozkazu odczytu wspolrzednych kartezjanskich
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV; // ARM
//  the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;// W.S.
	the_robot->EDP_data.get_arm_type = MOTOR;

	the_robot->create_command();
	// Sprawdzenie warunku poczatkowego - reakcji operatora
	if (  ecp_t.operator_reaction ("Next motion?") ) {
		// Oczekiwanie skonczone
		// wyslanie rozkazu odczytu
		the_robot->communicate = true; // wlaczamy robota na jeden krok
		the_robot->execute_motion();
		the_robot->communicate = false; // wylaczamy robota
		// aktualizacja stanu the_robot na podstawie danych przyslanych z EDP
		the_robot->get_reply();
		if (!is_supplementary_list_head()) { // Czy wskaznik na glowe listy jest NULL
			// Tworzymy glowe listy
//      create_supplementary_list_head(XYZ_EULER_ZYZ, 0.0, the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates); // W.S.
			create_supplementary_list_head(MOTOR, 0.0, the_robot->EDP_data.current_motor_arm_coordinates);
		} else {
			// Wstaw do listy nowa pozycje
//      insert_supplementary_list_element(XYZ_EULER_ZYZ, 0.0, the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates);
			insert_supplementary_list_element(MOTOR, 0.0, the_robot->EDP_data.current_motor_arm_coordinates);
		}
		return false;
	} else {
		// Czekamy dalej
		return true;
	}
} // end: irp6ot_operator_reaction_condition::condition_value ()
// --------------------------------------------------------------------------
