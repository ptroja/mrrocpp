 // -------------------------------------------------------------------------
//                            ecp_g_smooth2.cc
//            Effector Control Process (lib::ECP) - smooth2 generator
// Generator powstal na podstawie generatora smooth, glowna zmiana jest
// rezygnacja z podawanie predkosci poczatkowej i koncowej w kazdym ruchu.
// W przeciwienstwie do smootha generator smooth2 nie dopuszcza nigdy do przekroczenia
// maksymalnej podanej predkosci ruchu dla danej osi
// autor: Rafal Tulwin
// Ostatnia modyfikacja: 2009
// -------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth2.h"
#include <fstream>
#include <string.h>
#include "ecp/common/ecp_smooth2_taught_in_pose.h"
//#include "lib/y_math.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

void smooth2::set_relative(void) {
	type=2;
}

void smooth2::set_absolute(void) {
	type=1;
}

bool smooth2::eq(double a, double b) {
	const double EPS = 0.0001;
	const double& diff = a - b;
	return diff < EPS && diff > -EPS;
}

bool smooth2::load_file_with_path(char* file_name) {

    // Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,

    char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
    lib::POSE_SPECIFICATION ps;     // Rodzaj wspolrzednych
    uint64_t e;       // Kod bledu systemowego
    uint64_t number_of_poses; // Liczba zapamietanych pozycji
    uint64_t i, j;    // Liczniki petli
    bool first_time = true; // Znacznik
    int extra_info;
    double v[MAX_SERVOS_NR];
    double a[MAX_SERVOS_NR];	// Wczytane wspolrzedne
    double coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne

    ifstream from_file(file_name); // otworz plik do odczytu
    if (!from_file)
    {
        perror(file_name);
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    if ( !(from_file >> coordinate_type) )
    {
        from_file.close();
        throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
    }

    // Usuwanie spacji i tabulacji
    i = 0;
    j = 0;
    while ( coordinate_type[i] == ' ' || coordinate_type[i] == '\t')
        i++;
    while ( coordinate_type[i] != ' '   && coordinate_type[i] != '\t' &&
            coordinate_type[i] != '\n'  && coordinate_type[i] != '\r' &&
            coordinate_type[j] != '\0' )
    {
        coordinate_type[j] = toupper(coordinate_type[i]);
        i++;
        j++;
    }
    coordinate_type[j] = '\0';

    if ( !strcmp(coordinate_type, "MOTOR") )
    {
        ps = lib::MOTOR;
    }
    else if ( !strcmp(coordinate_type, "JOINT") )
    {
        ps = lib::JOINT;
    }
    else if ( !strcmp(coordinate_type, "XYZ_ANGLE_AXIS") )
    {
        ps = lib::XYZ_ANGLE_AXIS;
    }
    else if ( !strcmp(coordinate_type, "XYZ_EULER_ZYZ") )
    {
        ps = lib::XYZ_EULER_ZYZ;
    }

    else
    {
        from_file.close();
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
    }

    // printf("po coord type %d\n", ps);
    if ( !(from_file >> number_of_poses) )
    {
        from_file.close();
        throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
    }

    // printf("po number of poses %d\n", number_of_poses);
    flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
    // printf("po flush pose list\n");


    for ( i = 0; i < number_of_poses; i++)
    {
        //printf("w petli\n");
        // printf("po vk\n");

        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> v[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                from_file.close();
                throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }

        // printf("po v\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> a[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                from_file.close();
                throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }

        // printf("po a\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> coordinates[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                from_file.close();
                throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }

        if (first_time)
        {
            // Tworzymy glowe listy
            first_time = false;
            create_pose_list_head(ps, v, a, coordinates);
        }
        else
        {
            // Wstaw do listy nowa pozycje
            insert_pose_list_element(ps, v, a, coordinates);
            // printf("Pose list element: %d, %f, %f, %f, %f\n", ps, vp[0], vk[0], v[0], a[0]);
        }
    } // end: for
    from_file.close();

    return true;
} // end: load_file_with_path()

void smooth2::reset() {
	flush_pose_list();
	flush_coordinate_list();
	first_coordinate = true;
	first_interval = true;
	trajectory_generated = false;
	trajectory_calculated = false;
	rec = false;
}

void smooth2::flush_pose_list(void) {
   if (pose_list != NULL) {
       pose_list->clear();
	   first_coordinate=true;
   }
}

// -------------------------------------------------------return iterator to beginning of the list
void smooth2::initiate_pose_list(void) {
    pose_list_iterator = pose_list->begin();
}
// -------------------------------------------------------move pose_list iterator to the next position
void smooth2::next_pose_list_ptr(void) {
    if (pose_list_iterator != pose_list->end()) {
        pose_list_iterator++;
    }
}

void smooth2::prev_pose_list_ptr(void) {
	if (pose_list_iterator != pose_list->begin()) {
		pose_list_iterator--;
	}
}

bool smooth2::is_last_list_element(void) {
    // sprawdza czy aktualnie wskazywany element listy ma nastepnik
    // jesli <> nulla
    if ( pose_list_iterator != pose_list->end() )
    {
        if ( (++pose_list_iterator) != pose_list->end() )
        {
            --pose_list_iterator;
            return false;
        }
        else
        {
            --pose_list_iterator;
            return true;
        }
        ; // end if
    }
    return false;
}

void smooth2::create_pose_list_head (lib::POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR]) {
    pose_list->push_back(ecp_smooth2_taught_in_pose(ps, v, a, coordinates));
    pose_list_iterator = pose_list->begin();
}

void smooth2::insert_pose_list_element (lib::POSE_SPECIFICATION ps, double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR]) {
    pose_list->push_back(ecp_smooth2_taught_in_pose(ps, v, a, coordinates));
    pose_list_iterator++;
}

int smooth2::pose_list_length(void) {
    return pose_list->size();
}

void smooth2::initiate_coordinate_list(void) {

	if (coordinate_list != NULL) {
		coordinate_list_iterator = coordinate_list->begin();
	}
}

void smooth2::flush_coordinate_list(void) {

	if (coordinate_list != NULL) {
		coordinate_list->clear();
	}
}

bool smooth2::load_a_v_min (char* file_name)
{
    uint64_t e;       // Kod bledu systemowego
    uint64_t j;    // Liczniki petli
    std::ifstream from_file(file_name); // otworz plik do odczytu

    if (!from_file)
    {
        // printf("error\n");
        perror(file_name);
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    if ( !(from_file >> v_grip_min) )
    { // Zabezpieczenie przed danymi nienumerycznymi
        from_file.close();
        throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
    }

    from_file.close();
    return true;
} // end: bool load_a_v_min()

bool smooth2::load_a_v_max (char* file_name)
{
    uint64_t e;       // Kod bledu systemowego
    uint64_t j;    // Liczniki petli
    std::ifstream from_file(file_name); // otworz plik do odczytu

    if (!from_file)
    {
        // printf("error\n");
        perror(file_name);
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_motor[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_motor[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_joint[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_joint[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_zyz[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_zyz[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_aa[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_aa[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    from_file.close();
    return true;
} // end: bool load_a_v_max()

smooth2::smooth2 (common::task::task& _ecp_task, bool _is_synchronised)
        :
        delta (_ecp_task), debug(false),first_coordinate(true)
{

    int i;
    double v[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double a[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

	pose_list = new std::list<ecp_smooth2_taught_in_pose>();
	coordinate_list = new std::list<coordinates>();

	trajectory_generated = false;
	trajectory_calculated = false;
	rec = false;

	distance_eps = 0.00001;

	int size = 1 + strlen(ecp_t.mrrocpp_network_path) + strlen("data/a_v_max.txt");
	char * path1 = new char[size];
	// Stworzenie sciezki do pliku.
	strcpy(path1, ecp_t.mrrocpp_network_path);
	sprintf(path1, "%sdata/a_v_max.txt"	, ecp_t.mrrocpp_network_path);

	size = 1 + strlen(ecp_t.mrrocpp_network_path) + strlen("data/a_v_min.txt");
	char * path2 = new char[size];
	// Stworzenie sciezki do pliku.
	strcpy(path2, ecp_t.mrrocpp_network_path);
	sprintf(path2, "%sdata/a_v_min.txt", ecp_t.mrrocpp_network_path);

	load_a_v_max(path1);
	load_a_v_min(path2);

    delete[] path1;
    delete[] path2;

    is_synchronised = _is_synchronised;
	  type=1;
    //v_grip_min=5;
       /*for (int g = 0; g < 8; g++) {
      v_k[g] = 0.0;
      v_p[g] = 0.0;
    }*/

} // end : konstruktor

double smooth2::generate_next_coords(int node_counter, int interpolation_node_no, double start_position, double v_p, double v_r,
									double v_k, double a_r, int k, double przysp, double jedn, double s_przysp, double s_jedn) {

    //funkcja obliczajaca polozenie w danym makrokroku

	double next_position;

    double tk=10*STEP;

        if(node_counter < przysp) { //pierwszy etap
            if(v_p <= v_r) { //przyspieszanie w pierwszym etapie
            	//printf("start pos: %f\t node counter: %d\n", start_position, node_counter);
            	//printf(" przysp1 %d ", node_counter);
                next_position = start_position +
                                   k*(node_counter*v_p*tk + node_counter*node_counter*a_r*tk*tk/2);

            } else { //hamowanie w pierwszym etapie
            	//printf(" ham1 %d ", node_counter);
                next_position = start_position +
                                   k*(node_counter*tk*v_p -
                                         node_counter*node_counter*tk*tk*a_r/2);
            }
            //printf("%f\t", next_position);
        } else if(node_counter <= jedn) { // drugi etap - ruch jednostajny
        	//printf(" jedn %d ", node_counter);
            next_position = start_position +
                               k*(s_przysp + ((node_counter - przysp)*tk)*v_r);
            //printf("%f\t", next_position);

        } else if(node_counter <= interpolation_node_no) { //trzeci etap

        	if(v_k <= v_r) { //hamowanie w trzecim etapie
            	//printf(" ham2 %d ", node_counter);
                next_position = start_position +
                                   k*(s_przysp + s_jedn +
                                         ((node_counter - jedn) * tk ) * v_r -
                                         ((node_counter - jedn) * tk) * ((node_counter - jedn) * tk) * a_r / 2);
                //printf("next pos: %f\t node: %d\t", next_position, node_counter);
            } else { //przyspieszanie w trzecim etapie
            	//printf(" przysp2 %d ", node_counter);
                next_position = start_position +
                                   k*(s_przysp + s_jedn +
                                         ((node_counter - jedn) * tk) * v_r +
                                         a_r * ((node_counter - jedn) * tk) * ((node_counter - jedn) * tk) / 2);
            }
            //printf("%f\t", next_position);
        }

    //printf("next pos: %f\t node: %d\t", next_position, node_counter);
    return next_position;
}

void smooth2::generate_cords() {

	double coordinate[MAX_SERVOS_NR];
	int private_node_counter = 1;
	initiate_pose_list();
	flush_coordinate_list();
	for (int j = 0; j < pose_list_length(); j++) {

		for (int z = 0; z < pose_list_iterator->interpolation_node_no; z++) {
			for (int i = 0; i < MAX_SERVOS_NR; i++) {

				if (fabs(pose_list_iterator->start_position[i] - pose_list_iterator->coordinates[i]) < distance_eps) {
					coordinate[i] = pose_list_iterator->start_position[i]; //dla drogi 0
				} else {
					coordinate[i] = generate_next_coords(private_node_counter,
							pose_list_iterator->interpolation_node_no,
							pose_list_iterator->start_position[i],
							pose_list_iterator->v_p[i],
							pose_list_iterator->v_r[i],
							pose_list_iterator->v_k[i],
							pose_list_iterator->a_r[i],
							pose_list_iterator->k[i],
							pose_list_iterator->przysp[i],
							pose_list_iterator->jedn[i],
							pose_list_iterator->s_przysp[i],
							pose_list_iterator->s_jedn[i]);
				}
			}
			private_node_counter++;
			coordinate_list->push_back(coordinates(coordinate));
		}
		private_node_counter = 1;
		next_pose_list_ptr();
	}

	//printowanie listy coordinate
	initiate_coordinate_list();
	for (int m = 0; m < coordinate_list->size(); m++) {
		printf("%f\t", coordinate_list_iterator->coordinate[8]);
		coordinate_list_iterator++;
	}
	printf("\ngenerate_cords\n");
}

//set necessary instructions, and other data for preparing the robot
bool smooth2::first_step() { //wywolywane tylko raz w calej trajektorii

    initiate_pose_list();
    td.arm_type = pose_list_iterator->arm_type;

    first_interval=true;

    switch ( td.arm_type ) {

    case lib::MOTOR:
        the_robot->EDP_data.instruction_type = lib::GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = lib::MOTOR;
        the_robot->EDP_data.get_arm_type = lib::MOTOR;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        break;
    case lib::JOINT:
        the_robot->EDP_data.instruction_type = lib::GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = lib::JOINT;
        the_robot->EDP_data.get_arm_type = lib::JOINT;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        break;
    case lib::XYZ_EULER_ZYZ:
        the_robot->EDP_data.instruction_type = lib::GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
        the_robot->EDP_data.get_arm_type = lib::XYZ_EULER_ZYZ;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        break;
    case lib::XYZ_ANGLE_AXIS:
        the_robot->EDP_data.instruction_type = lib::GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
        the_robot->EDP_data.get_arm_type = lib::XYZ_ANGLE_AXIS;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        break;
    default:
        throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    } // end : switch ( td.arm_type )

    return true;
} // end: bool ecp_smooth2_generator::first_step ( )

bool smooth2::next_step () {

    int i; //licznik petli

    if (!trajectory_generated && !trajectory_calculated) {
    	calculate(); //wypelnienie pozostalych zmiennych w liscie pose_list
    	generate_cords(); //wypelnienie listy coordinate_list (lista kolejnych pozycji w makrokrokach)
    	trajectory_generated = true;
    	initiate_pose_list(); //ustawienie iteratora pose_list na poczatek
    	initiate_coordinate_list(); //ustawienie iteratora coordinate_list na poczatek
    	printf("trajektoria wygenerowana\n");
    }

    if (!trajectory_generated || !trajectory_calculated) {
    	return false;
    }

    if (node_counter-1 == pose_list_iterator->interpolation_node_no) {//czy poprzedni makrokrok byl ostatnim

    	if(is_last_list_element()) { //ostatni punkt (koniec listy pozycji pose_list)

    		return false;

    	} else {//lista pozycji pose_list nie jest skonczona wiec idziemy do nastepnego punktu

			node_counter = 0; //ustawienie makrokroku na 1 (jest za chwile inkrementowany przez move())
		    next_pose_list_ptr();
		    td.interpolation_node_no = pose_list_iterator->interpolation_node_no;
		}
    } else {

    	the_robot->EDP_data.instruction_type = lib::SET; //ustawienie parametrow ruchu w edp_data
    	the_robot->EDP_data.get_type = NOTHING_DV; //ponizej w caseach jest dalsze ustawianie
    	the_robot->EDP_data.get_arm_type = lib::INVALID_END_EFFECTOR;

    	switch ( td.arm_type ) {
    		case lib::XYZ_EULER_ZYZ:

    			the_robot->EDP_data.instruction_type = lib::SET; //dalsze ustawianie parametrow ruchu w edp
    			the_robot->EDP_data.set_type = ARM_DV; // ARM
    			the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
    			the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    			the_robot->EDP_data.next_interpolation_type = lib::MIM;
    			the_robot->EDP_data.motion_steps = td.internode_step_no;
    			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    			for (i = 0; i < 6; i++) {//zapisanie nastepnego polazenia (makrokroku) do robota
    			    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = coordinate_list_iterator->coordinate[i];
    			}

    			coordinate_list_iterator++;

    			//gripper

    			if(pose_list_iterator->v_grip*node_counter < pose_list_iterator->coordinates[6]) {
    			    the_robot->EDP_data.next_gripper_coordinate = pose_list_iterator->v_grip*node_counter;
    			} else {
    			    the_robot->EDP_data.next_gripper_coordinate = pose_list_iterator->coordinates[6];
    			}
    			//the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;//TODO to jest tymczasowe wiec trzeba poprawic

    			break;

    		case lib::XYZ_ANGLE_AXIS:

    			the_robot->EDP_data.instruction_type = lib::SET; //dalsze ustawianie parametrow ruchu w edp
    			the_robot->EDP_data.set_type = ARM_DV; // ARM
    			the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
    			the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    			the_robot->EDP_data.next_interpolation_type = lib::MIM;
    			the_robot->EDP_data.motion_steps = td.internode_step_no;
    			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    			for (i = 0; i < 6; i++) {//zapisanie nastepnego polazenia (makrokroku) do robota
    			    the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = coordinate_list_iterator->coordinate[i];
    			}

    			coordinate_list_iterator++;

    			//gripper

    			if(pose_list_iterator->v_grip*node_counter < pose_list_iterator->coordinates[6]) {
    			    the_robot->EDP_data.next_gripper_coordinate = pose_list_iterator->v_grip*node_counter;
    			} else {
    			    the_robot->EDP_data.next_gripper_coordinate = pose_list_iterator->coordinates[6];
    			}
    			//the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;//TODO to jest tymczasowe wiec trzeba poprawic

    			break;

    		case lib::JOINT:

    			the_robot->EDP_data.instruction_type = lib::SET;
    		    the_robot->EDP_data.set_type = ARM_DV; // ARM
    		    the_robot->EDP_data.set_arm_type = lib::JOINT;
    		    the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    		    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    		    the_robot->EDP_data.motion_steps = td.internode_step_no;
    		    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    		    for (i=0; i < MAX_SERVOS_NR; i++) {
    		        the_robot->EDP_data.next_joint_arm_coordinates[i] = coordinate_list_iterator->coordinate[i];
    		    }

    		    coordinate_list_iterator++;

    		    //gripper

    		    if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
    		        i=7;
    		    } else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
    		        i=6;
				}

    		    if(pose_list_iterator->v_grip*node_counter < coordinate_list_iterator->coordinate[i]) {
    		    	the_robot->EDP_data.next_joint_arm_coordinates[i] = pose_list_iterator->v_grip*node_counter;
    		    } else {
    		    	the_robot->EDP_data.next_joint_arm_coordinates[i] = pose_list_iterator->coordinates[i];
    		    }

    		    //the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;//TODO to jest tymczasowe wiec trzeba poprawic

    		    break;

    		case lib::MOTOR:

    			the_robot->EDP_data.instruction_type = lib::SET;
    		    the_robot->EDP_data.set_type = ARM_DV; // ARM
    		    the_robot->EDP_data.set_arm_type = lib::MOTOR;
    		    the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    		    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    		    the_robot->EDP_data.motion_steps = td.internode_step_no;
    		    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    		    for (i=0; i < MAX_SERVOS_NR; i++) {
    		        the_robot->EDP_data.next_motor_arm_coordinates[i] = coordinate_list_iterator->coordinate[i];
    		    }

    		    coordinate_list_iterator++;

    		    //gripper

    		    if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
    		        i=7;
    		    } else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
    		        i=6;
				}

    		    if(pose_list_iterator->v_grip*node_counter < coordinate_list_iterator->coordinate[i]) {
    		    	the_robot->EDP_data.next_motor_arm_coordinates[i] = pose_list_iterator->v_grip*node_counter;
    		    } else {
    		    	the_robot->EDP_data.next_motor_arm_coordinates[i] = pose_list_iterator->coordinates[i];
    		    }

    		    //the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;//TODO to jest tymczasowe wiec trzeba poprawic

    		    break;

    		default:
    			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    	}// end:switch
    }// end: if
    return true;

} // end: bool ecp_smooth2_generator::next_step ( )

void smooth2::calculate(void) {

	double s[MAX_SERVOS_NR];//droga w danych stopniach swobody w jednym ruchu
	double s_temp1[MAX_SERVOS_NR], s_temp2[MAX_SERVOS_NR];
	double t_temp1, t_temp2;
    double t[MAX_SERVOS_NR];
    double t_max; //nadluzszy czas ruchu w jednej osi w jednym ruchu
    int i;
    double tk = 10 * STEP; //czas jednego makrokroku
    int gripp; //os grippera

    double v_r_next[MAX_SERVOS_NR];//predkosc kolejnego ruchu

    double temp;//generalny temp do wszystkiego

    initiate_pose_list();//ustawianie wskaznika listy pozycji na poczatek

    td.internode_step_no = 10;
    td.value_in_step_no = td.internode_step_no - 2;

    for (int j = 0; j < pose_list_length(); j++) {
    	printf("petla listy pozycji %d\n", j);
		td.arm_type = pose_list_iterator->arm_type;

		if (first_interval) {//pierwsza pozycja ruchu
			//wywoluje sie tylko raz, przy pierwzszym ruchu w trajektorii, musi byc rozroznione, gdyz tutaj v_p jest = 0 a pozycja jest
			// ...odczytywana z ramienia...

			switch (td.arm_type) {

			case lib::XYZ_EULER_ZYZ:
				gripp = 6;
				//printf("euler w first_interval\n");
				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					pose_list_iterator->start_position[i] = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];//pierwsze przypisanie start_position
					if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}
				temp = pose_list_iterator->coordinates[gripp];
				pose_list_iterator->start_position[gripp] = the_robot->EDP_data.current_gripper_coordinate;
				if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_list_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}
				pose_list_iterator->start_position[7] = 0.0;

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					//printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					//printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					if (rec == false) {
						pose_list_iterator->v_r[i] = v_max_zyz[i] * pose_list_iterator->v[i];
					}

					pose_list_iterator->a_r[i] = a_max_zyz[i] * pose_list_iterator->a[i];
					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if(pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
						//printf("ustawienie k w osi %d na -1\n", i);
						pose_list_iterator->k[i] = -1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
			        }
			        else {
			        	//printf("ustawienie k w osi %d na 1\n", i);
			        	pose_list_iterator->k[i] = 1;
			        }
				}

				break;

			case lib::XYZ_ANGLE_AXIS:
				gripp = 6;

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					pose_list_iterator->start_position[i] = the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];//pierwsze przypisanie start_position
					if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}
				temp = pose_list_iterator->coordinates[gripp];
				pose_list_iterator->start_position[gripp] = the_robot->EDP_data.current_gripper_coordinate;
				if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_list_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}
				pose_list_iterator->start_position[7] = 0.0;

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					//printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					//printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					if (rec == false) {
						pose_list_iterator->v_r[i] = v_max_aa[i] * pose_list_iterator->v[i];
					}
					pose_list_iterator->a_r[i] = a_max_aa[i] * pose_list_iterator->a[i];
					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if(pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
						//printf("ustawienie k w osi %d na -1\n", i);
						pose_list_iterator->k[i] = -1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
			        }
			        else {
			        	//printf("ustawienie k w osi %d na 1\n", i);
			        	pose_list_iterator->k[i] = 1;
			        }
				}

				break;

			case lib::JOINT:
				if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
					gripp=7;
				} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
					gripp=6;
				}

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					pose_list_iterator->start_position[i] = the_robot->EDP_data.current_joint_arm_coordinates[i];//pierwsze przypisanie start_position
					if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}
				temp = pose_list_iterator->coordinates[gripp];
				pose_list_iterator->start_position[gripp] = the_robot->EDP_data.current_joint_arm_coordinates[gripp];
				if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_list_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}
				if (gripp < (MAX_SERVOS_NR -1) ) {
					pose_list_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					//printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					//printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					if (rec == false) {
						pose_list_iterator->v_r[i] = v_max_joint[i] * pose_list_iterator->v[i];
					}
					pose_list_iterator->a_r[i] = a_max_joint[i] * pose_list_iterator->a[i];
					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if(pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
						//printf("ustawienie k w osi %d na -1\n", i);
						pose_list_iterator->k[i] = -1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
					} else {
					//printf("ustawienie k w osi %d na 1\n", i);
					pose_list_iterator->k[i] = 1;
					}
				}

				break;

			case lib::MOTOR:
				if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
					gripp=7;
				} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
					gripp=6;
				}

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					pose_list_iterator->start_position[i] = the_robot->EDP_data.current_motor_arm_coordinates[i];//pierwsze przypisanie start_position
					if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}
				temp = pose_list_iterator->coordinates[gripp];
				pose_list_iterator->start_position[gripp] = the_robot->EDP_data.current_motor_arm_coordinates[gripp];
				if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_list_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}
				if (gripp < (MAX_SERVOS_NR -1) ) {
					pose_list_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					//printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					//printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					if (rec == false) {
						pose_list_iterator->v_r[i] = v_max_motor[i] * pose_list_iterator->v[i];
					}
					pose_list_iterator->a_r[i] = a_max_motor[i] * pose_list_iterator->a[i];
					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if(pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
						//printf("ustawienie k w osi %d na -1\n", i);
						pose_list_iterator->k[i] = -1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
					} else {
					//printf("ustawienie k w osi %d na 1\n", i);
					pose_list_iterator->k[i] = 1;
					}
				}

				break;

			default:
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);

			} // end: switch

			first_interval = false;

		} else { // end: if(first_interval)
			switch (td.arm_type) {
			//tutaj jestesmy jeszcze ciagle w poprzedniej pozycji pose_list
			case lib::XYZ_EULER_ZYZ:
				gripp = 6;
				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr();					//predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

				pose_list_iterator->start_position[gripp] = the_robot->EDP_data.current_gripper_coordinate;
				pose_list_iterator->start_position[7] = 0.0;

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					//printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					//printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					if (rec == false) {
						pose_list_iterator->v_r[i] = v_max_zyz[i] * pose_list_iterator->v[i];
					}
					pose_list_iterator->a_r[i] = a_max_zyz[i] * pose_list_iterator->a[i];
				}

				break;

			case lib::XYZ_ANGLE_AXIS:
				gripp = 6;
				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr();					//predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

				pose_list_iterator->start_position[gripp] = the_robot->EDP_data.current_gripper_coordinate;
				pose_list_iterator->start_position[7] = 0.0;

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					//printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					//printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					if (rec == false) {
						pose_list_iterator->v_r[i] = v_max_aa[i] * pose_list_iterator->v[i];
					}
					pose_list_iterator->a_r[i] = a_max_aa[i] * pose_list_iterator->a[i];
				}

				break;

			case lib::JOINT:

				if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
					gripp=7;
				} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
					gripp=6;
				}

				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr();					//predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

				pose_list_iterator->start_position[gripp] = the_robot->EDP_data.current_joint_arm_coordinates[gripp];
				if (gripp < (MAX_SERVOS_NR -1)) {
					pose_list_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					//printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					//printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					if (rec == false) {
						pose_list_iterator->v_r[i] = v_max_joint[i] * pose_list_iterator->v[i];
					}
					pose_list_iterator->a_r[i] = a_max_joint[i] * pose_list_iterator->a[i];
				}

				break;

			case lib::MOTOR:

				if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
					gripp=7;
				} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
					gripp=6;
				}

				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr();					//predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

				pose_list_iterator->start_position[gripp] = the_robot->EDP_data.current_motor_arm_coordinates[gripp];
				if (gripp < (MAX_SERVOS_NR -1)) {
					pose_list_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					//printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					//printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					if (rec == false) {
						pose_list_iterator->v_r[i] = v_max_motor[i] * pose_list_iterator->v[i];
					}
					pose_list_iterator->a_r[i] = a_max_motor[i] * pose_list_iterator->a[i];
				}

				break;

			default:
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
		} //end else (first interval)

		for(i=0;i<MAX_SERVOS_NR;i++) {//zapisanie coordinate_delta
			td.coordinate_delta[i] = pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i];
			//printf("delta : coordinates %f i start_position %f\n",pose_list_iterator->coordinates[i], pose_list_iterator->start_position[i]);
		}

		for (i = 0; i < MAX_SERVOS_NR; i++) {//zapisanie v_r_next i k dla nastepnego ruchu
			if (is_last_list_element()) {
				//printf("ostatni ruch\n");
				v_r_next[i] = 0.0;
				//pose_list_iterator->v_k[i] = 0; //to nie jest potrzebne tak naprawde...
			} else {
				temp = pose_list_iterator->k[i];
				next_pose_list_ptr();

				//printf("coordinates %f i start_position %f\n",pose_list_iterator->coordinates[i], pose_list_iterator->start_position[i]);
				if (eq(pose_list_iterator->coordinates[i],pose_list_iterator->start_position[i])) {
					pose_list_iterator->k[i] = -temp;//jesli droga jest rowna 0 w nastepnym ruchu to kierunek ustawiany jest na przeciwny aby robot zwolnil do 0
				} else if(pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i] < 0) {//nadpisanie k dla nastepnego ruchu
					//printf("nadpisanie k dla ruchu nastepnego w osi %d na -1\n", i);
					pose_list_iterator->k[i] = -1;
				} else {
					//printf("nadpisanie k dla ruchu nastepnego w osi %d na 1\n", i);
		        	pose_list_iterator->k[i] = 1;
		        }

				if (pose_list_iterator->k[i] != temp) {
					//printf("ustawianie v_r_next w osi %d\n", i);
					v_r_next[i] = 0;
				} else {
					v_r_next[i] = v_max_zyz[i] * pose_list_iterator->v[i];
				}
				//printf("ruch nieostatni, nastepna predkosc: %f\n", v_max_zyz[i] * pose_list_iterator->v[i]);
				prev_pose_list_ptr();
			}
		}
		// ==================================== poczatek oliczen ========================================

		t_max = 0;

		//pose_list_iterator->start_position[1] = -0.288; //tymczasowa opcja (powrot do pozycji synchro)

		//obliczenie drogi dla kazdej osi
		for (i = 0; i < MAX_SERVOS_NR; i++) {
			s[i]=fabs(pose_list_iterator->coordinates[i] - pose_list_iterator->start_position[i]);
			//s_zero[i] = 0; //wyzerowanie drogi w zerowym etapie (domyslnie bez tego etapu)
			//printf("droga dla osi %d = %f\n", i, s[i]);
		}
		//sprawdzanie czy droga w etapach przyspieszenia i hamowania nie jest wieksza niz droga calego ruchu

		//warunki na modele ruchu dla wszystkich osi
		for (i = 0; i < MAX_SERVOS_NR; i++) { //petla w ktorej obliczany jest czas dla kazdej osi i sprawdzane jest czy da sie wykonac ruch w zalozonych etapach

			if (s[i] < distance_eps) {//najmniejsza wykrywalna droga
				//printf("droga 0 %d\n", i);
				pose_list_iterator->model[i] = 0; //model 0... brak ruchu
				t[i] = 0;
				pose_list_iterator->s_przysp[i] = 0;
				pose_list_iterator->s_jedn[i] = 0;
				pose_list_iterator->v_k[i] = 0;
				continue;
			}

			if (pose_list_iterator->v_p[i] < pose_list_iterator->v_r[i] && v_r_next[i] < pose_list_iterator->v_r[i]) { //pierwszy model
				printf("pierwszy model w osi %d\n", i);

				pose_list_iterator->model[i] = 1;

				s_temp1[i] = pose_list_iterator->v_p[i] * (pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i] + (0.5 * pose_list_iterator->a_r[i] * ((pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i]) * ((pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i]));
				s_temp2[i] = pose_list_iterator->v_r[i] * (pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i] - (0.5 * pose_list_iterator->a_r[i] * ((pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i]) * ((pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i]));

				t_temp1 = (pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i];
				t_temp2 = (pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i];

				pose_list_iterator->v_k[i] = v_r_next[i];

				printf("s_temp1: %f\ts_temp2: %f\n", s_temp1[i], s_temp2[i]);
				printf("t_temp1: %f\tt_temp2: %f\n", t_temp1, t_temp2);

				if (s_temp1[i] + s_temp2[i] > s[i]) {
					printf("redukcja predkosci w osi %d\n", i);
					t[i] = t_temp1 + t_temp2;
					pose_list_iterator->t = t[i];
					reduction_model_1(pose_list_iterator, i, s[i]);

					if (trajectory_calculated == true) {
						return;
					}

				} else {//droga przyspieszenia i opoznienia nie przekracza drogi ruchu

					t[i] = t_temp1 + t_temp2 + (s[i] - (s_temp1[i] + s_temp2[i]))/pose_list_iterator->v_r[i];
					pose_list_iterator->s_przysp[i] = s_temp1[i];
					pose_list_iterator->s_jedn[i] = s[i] - s_temp2[i] - s_temp1[i];
				}

            } else if (pose_list_iterator->v_p[i] < pose_list_iterator->v_r[i] && (v_r_next[i] > pose_list_iterator->v_r[i] || eq(v_r_next[i], pose_list_iterator->v_r[i]))) { // drugi model
            	printf("drugi model w osi %d\n", i);

            	pose_list_iterator->model[i] = 2;

            	s_temp1[i] = pose_list_iterator->v_p[i] * (pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i] + (0.5 * pose_list_iterator->a_r[i] * ((pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i]) * ((pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i]));
        		t_temp1 = (pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i];

            	pose_list_iterator->v_k[i] = pose_list_iterator->v_r[i];

				printf("s_temp1: %f\n", s_temp1[i]);
				printf("t_temp1: %f\n", t_temp1);

            	if (s_temp1[i] > s[i]) {
            		printf("redukcja predkosci w osi %d\n", i);
					t[i] = t_temp1;
					vk_reduction(pose_list_iterator, i, s[i], t[i]);

					if (trajectory_calculated == true) {
						return;
					}

            	} else {

            		t[i] = t_temp1 + (s[i] - s_temp1[i])/pose_list_iterator->v_r[i];
    				pose_list_iterator->s_przysp[i] = s_temp1[i];
    				pose_list_iterator->s_jedn[i] = s[i] - s_temp1[i];
            	}

			} else if (eq(pose_list_iterator->v_p[i], pose_list_iterator->v_r[i]) && (v_r_next[i] > pose_list_iterator->v_r[i] || eq(v_r_next[i], pose_list_iterator->v_r[i]))) { //trzeci model
				printf("trzeci model w osi %d\n", i);

				pose_list_iterator->model[i] = 3;

				t[i] = s[i]/pose_list_iterator->v_r[i];

				pose_list_iterator->s_przysp[i] = 0;
				pose_list_iterator->s_jedn[i] = s[i];

				pose_list_iterator->v_k[i] = pose_list_iterator->v_r[i];

			} else if (eq(pose_list_iterator->v_p[i], pose_list_iterator->v_r[i]) && v_r_next[i] < pose_list_iterator->v_r[i]) { //czwarty model
				printf("czwarty model w osi %d\n", i);

				pose_list_iterator->model[i] = 4;

				s_temp1[i] = pose_list_iterator->v_r[i] * (pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i] - (0.5 * pose_list_iterator->a_r[i] * ((pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i]) * ((pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i]));
				t_temp1 = (pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i];

				pose_list_iterator->v_k[i] = v_r_next[i];

				printf("s_temp1: %f\n", s_temp1[i]);
				printf("t_temp1: %f\n", t_temp1);

				if (s_temp1[i] > s[i]) {
					printf("redukcja predkosci w osi %d\n", i);
					t[i] = t_temp1;
					vp_reduction(pose_list_iterator, i, s[i], t[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else {

					t[i] = t_temp1 + (s[i] - s_temp1[i])/pose_list_iterator->v_r[i];
					pose_list_iterator->s_przysp[i] = 0;
					pose_list_iterator->s_jedn[i] = s[i] - s_temp1[i];
				}

			} else {
				printf("\n ten przypadek nigdy nie moze wystapic\n");
			}

			if (t[i] > t_max) {//nadpisanie najdluzszego czasu
				t_max = t[i];
			}
		}

	    //kwantyzacja czasu
	    if(ceil(t_max/tk)*tk != t_max)//zaokraglenie czasu do wielokrotnosci trwania makrokroku
	    {
	        t_max = ceil(t_max/tk);
	        t_max = t_max*tk;
	        //printf("t_max = %f\n", t_max);
	        pose_list_iterator->t = t_max;
	    }

		pose_list_iterator->interpolation_node_no = (int)round(t_max / tk);
		printf("liczba makrokrokow w ruchu %d\n", pose_list_iterator->interpolation_node_no);

		for (i = 0; i < MAX_SERVOS_NR; i++) {//obliczanie przysp i jedn a takze ewentualna redukcja predkosci z powodu zbyt krotkiego czasu

			//printf("czas ruchu w osi %d = %f\n", i, t[i]);

			if (s[i] < distance_eps || t[i] == 0) {//jesli droga jest mniejsza od najmniejszej wykrywalnej albo czas jest rowny 0
				//printf("droga 0 (koncowe obliczenia) w osi %d\n", i);
				pose_list_iterator->przysp[i] = 0;
				pose_list_iterator->jedn[i] = 0;
				continue;
			}

			if (fabs(t[i] - t_max) > tk) {//redukcja predkosci w danej osi ze wzgledu na zbyt krotki czas ruchu
				if (pose_list_iterator->model[i] == 1) { //model 1
					reduction_model_1(pose_list_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else if (pose_list_iterator->model[i] == 2) {//model 2
					reduction_model_2(pose_list_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else if (pose_list_iterator->model[i] == 3) {//model 3
					reduction_model_3(pose_list_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else if (pose_list_iterator->model[i] == 4) {//model 4
					reduction_model_4(pose_list_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else {
					printf(" ten przypadek nie moze wystapic (redukcja ze wzgledu na czas)\n");
				}

				printf("redukcja predkosci z powodu czasu w osi %d\n", i);
			}

			pose_list_iterator->przysp[i]=fabs((pose_list_iterator->v_r[i]-pose_list_iterator->v_p[i])/(pose_list_iterator->a_r[i]*tk));//zapisanie makrokroku w ktorym konczy sie przyspieszanie
			pose_list_iterator->jedn[i]=(t_max-(fabs(pose_list_iterator->v_r[i]-pose_list_iterator->v_k[i])/pose_list_iterator->a_r[i]))/tk;//zapisaine makrokroku w ktorym konczy sie jednostajny

		}

		//obliczanie v_grip
		pose_list_iterator->v_grip = (pose_list_iterator->coordinates[gripp]/pose_list_iterator->interpolation_node_no);
		if(pose_list_iterator->v_grip < v_grip_min) {
			pose_list_iterator->v_grip = v_grip_min;
		}

		int os = 1;
		printf("\n=============== pozycja trajektorii nr %d ==================\n", j);
		printf("czas ruchu %f\n", pose_list_iterator->t);
		printf("coordinates: %f\n", pose_list_iterator->coordinates[os]);
		printf("jedn: %f\t przysp: %f\n", pose_list_iterator->jedn[os], pose_list_iterator->przysp[os]);
		printf("start pos: %f\t kierunek (k): %f\n", pose_list_iterator->start_position[os], pose_list_iterator->k[os]);
		printf("s: %f\ns_przysp: %f\t s_jedn: %f\n", s[os], pose_list_iterator->s_przysp[os], pose_list_iterator->s_jedn[os]);
		printf("czas\t\tv_r\t\tv_r_next\ta_r\t\tv_p\t\tv_k\n");
		printf("%f\t%f\t%f\t%f\t%f\t%f\n", t[os], pose_list_iterator->v_r[os], v_r_next[os], pose_list_iterator->a_r[os], pose_list_iterator->v_p[os], pose_list_iterator->v_k[os]);
		printf("v_grip: %f\n\n", pose_list_iterator->v_grip);
    }
    trajectory_calculated = true;
    rec = false;
}//end - calculate

void smooth2::reduction_model_1(std::list<ecp_smooth2_taught_in_pose>::iterator pose_list_iterator, int i, double s) {
	printf("redukcja model 1\n");
	if (pose_list_iterator->v_p[i] < pose_list_iterator->v_k[i] && (pose_list_iterator->v_k[i] * pose_list_iterator->t
			- 0.5 * ((pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) *
					(pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]))/pose_list_iterator->a_r[i]) > s) {//proba dopasowania do modelu 2

		pose_list_iterator->model[i] = 2;
		printf("dopasowanie model 2 w redukcji modelu 1\n");
		reduction_model_2(pose_list_iterator, i, s);

	} else if (pose_list_iterator->v_p[i] > pose_list_iterator->v_k[i] && (pose_list_iterator->v_p[i] * pose_list_iterator->t
			- 0.5 * ((pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) *
					(pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]))/pose_list_iterator->a_r[i]) > s){ // proba dopasowania do modelu 4

		pose_list_iterator->model[i] = 4;
		printf("dopasowanie model 4 w redukcji modelu 1\n");
		reduction_model_4(pose_list_iterator, i, s);

	} else if (eq(pose_list_iterator->v_p[i], pose_list_iterator->v_k[i]) && pose_list_iterator->v_k[i] * pose_list_iterator->t > s) {//proba dopasowanie do modelu 3
		printf("dopasowanie model 3 w redukcji modelu 1\n");
		reduction_model_3(pose_list_iterator, i, s);

	} else { //normalna redukcja dla modelu 1
		printf("normalna redukcja model 1\n");

		double t1;//czas przyspieszania
		double t2;//czas jednostajnego
		double delta;//delta w rownaniu kwadratowym

		delta = (2 * pose_list_iterator->a_r[i] * pose_list_iterator->t + 2 * pose_list_iterator->v_k[i] + 2 * pose_list_iterator->v_p[i]) *
				(2 * pose_list_iterator->a_r[i] * pose_list_iterator->t + 2 * pose_list_iterator->v_k[i] + 2 * pose_list_iterator->v_p[i]) +
				8 * (- pose_list_iterator->v_p[i] * pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i] * pose_list_iterator->v_k[i] -
				2 * pose_list_iterator->a_r[i] * s);

		printf("delta: %f\n", delta);

		pose_list_iterator->v_r[i] = (-(2 * pose_list_iterator->a_r[i] * pose_list_iterator->t + 2 * pose_list_iterator->v_k[i] +
				                       2 * pose_list_iterator->v_p[i]) + sqrt(delta)) / (-4);

		t1 = fabs(pose_list_iterator->v_p[i] - pose_list_iterator->v_r[i]) / pose_list_iterator->a_r[i];
		t2 = pose_list_iterator->t - t1 - (fabs(pose_list_iterator->v_k[i] - pose_list_iterator->v_r[i]) / pose_list_iterator->a_r[i]);

		printf("t2: %f\n", t2);

		pose_list_iterator->s_przysp[i] = t1 * pose_list_iterator->v_p[i] +
										  0.5 * pose_list_iterator->a_r[i] * t1 * t1;
		pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * t2;
	}
}

void smooth2::reduction_model_2(std::list<ecp_smooth2_taught_in_pose>::iterator pose_list_iterator, int i, double s) {
	printf("redukcja model 2\n");
	//pierwszy stopien redukcji
	double a;

	a = (0.5 * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])
			+ pose_list_iterator->v_p[i] * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])
			- pose_list_iterator->v_k[i] * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) )
			/ (s - pose_list_iterator->v_k[i] * pose_list_iterator->t);

	if ((pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) / a > pose_list_iterator->t) { //drugi stopien redukcji
		if (s == pose_list_iterator->v_p[i] * pose_list_iterator->t) { //zabezpieczenie przed dzieleniem przez 0
			a = -1; //powoduje przejscie do nastepnego stopnia redukcji (moglaby byc dowolna ujemna liczba)
		} else {
			a = (0.5 * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])) /
				(s - pose_list_iterator->v_p[i]*pose_list_iterator->t);
		}
		printf("drugi stopien\n");
		if (a > pose_list_iterator->a_r[i] || a <= 0) {//trzeci stopien redukcji
			printf("trzeci stopien\n");
			double t1; //czas konca opoznienia
			double s1; // droga w etapie w ktorym redukujemy czas (przed etapem "odcinanym")
			double t2; //czas redukowanego kawalka

			t2 = pose_list_iterator->t - ((pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) / pose_list_iterator->a_r[i]);

			s1 = s - (pose_list_iterator->v_p[i] * (pose_list_iterator->t- t2)
			   + 0.5 * pose_list_iterator->a_r[i] * (pose_list_iterator->t - t2) * (pose_list_iterator->t - t2));

			if (pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i] * t2 * t2 //ujemna liczba pod pierwiastkiem, zabezpieczenie
					- 4 * pose_list_iterator->a_r[i] * (pose_list_iterator->v_p[i] * t2 - s1) < 0) {
				vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
				return;
			}

			t1 = (pose_list_iterator->a_r[i] * t2
					- (sqrt(pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i] * t2 * t2
					- 4 * pose_list_iterator->a_r[i] * (pose_list_iterator->v_p[i] * t2 - s1))) )
					/ (2 * pose_list_iterator->a_r[i]);

			if (pose_list_iterator->v_p[i] - pose_list_iterator->a_r[i] * t1 < 0) {//ujemna predkosc ruchu, zabezpieczenie
				vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
				return;
			}

			pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i] - pose_list_iterator->a_r[i] * t1;
			pose_list_iterator->s_przysp[i] = 0.5 * pose_list_iterator->a_r[i] * t1 * t1 + pose_list_iterator->v_r[i] * t1;
			pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * (t2 - 2 * t1);



			return;
		}

		pose_list_iterator->a_r[i] = a;
		pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i];
		pose_list_iterator->s_przysp[i] = 0;
		pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * (pose_list_iterator->t -
										(pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i]);

		return;
	}

	pose_list_iterator->a_r[i] = a;
	pose_list_iterator->v_r[i] = pose_list_iterator->v_k[i];
	pose_list_iterator->s_przysp[i] = pose_list_iterator->v_p[i] * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i]
	                                  + 0.5 * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])
	                                  * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) / pose_list_iterator->a_r[i];
	pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * (pose_list_iterator->t - (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) / pose_list_iterator->a_r[i]);
}

void smooth2::reduction_model_3(std::list<ecp_smooth2_taught_in_pose>::iterator pose_list_iterator, int i, double s) {
	printf("redukcja model 3\n");
	double t1; //czas konca opoznienia

	if(pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i] * pose_list_iterator->t * pose_list_iterator->t//liczba pierwiastkowana mniejsza od 0, zabezpieczenie
			- 4 * pose_list_iterator->a_r[i] * (pose_list_iterator->v_p[i] * pose_list_iterator->t - s) < 0) {
		vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
		return;
	}

	t1 = (pose_list_iterator->a_r[i] * pose_list_iterator->t
			- (sqrt(pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i] * pose_list_iterator->t * pose_list_iterator->t
			- 4 * pose_list_iterator->a_r[i] * (pose_list_iterator->v_p[i] * pose_list_iterator->t - s))) )
			/ (2 * pose_list_iterator->a_r[i]);

	if (pose_list_iterator->v_p[i] - pose_list_iterator->a_r[i] * t1 < 0) {//ujemna predkosc ruchu, zabezpieczenie
		vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
		return;
	}

	pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i] - pose_list_iterator->a_r[i] * t1;
	pose_list_iterator->s_przysp[i] = 0.5 * pose_list_iterator->a_r[i] * t1 * t1 + pose_list_iterator->v_r[i] * t1;
	pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * (pose_list_iterator->t - 2 * t1);
}


void smooth2::reduction_model_4(std::list<ecp_smooth2_taught_in_pose>::iterator pose_list_iterator, int i, double s) {
	printf("redukcja model 4\n");
	//pierwszy stopien redukcji
	double a;

	a = (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) * (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) /
		((-2) * (s - pose_list_iterator->v_p[i] * pose_list_iterator->t));

	if ((pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) / a > pose_list_iterator->t) { //drugi stopien redukcji
		if (s == pose_list_iterator->v_k[i] * pose_list_iterator->t) { //zabezpieczenie przed dzieleniem przez 0
			a = -1; //powoduje przejscie do nastepnego stopnia redukcji (moglaby byc dowolna ujemna liczba)
		} else {
			a = (0.5 * (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) * (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])) /
				(s - pose_list_iterator->v_k[i]*pose_list_iterator->t);
		}
		printf("drugi stopien\n");
		if (a > pose_list_iterator->a_r[i] || a <= 0) {//trzeci stopien redukcji
			double t1; //czas konca opoznienia (relatywny - liczac od poczatku redukowanego odcinka)
			double s1; // droga w etapie w ktorym redukujemy czas (po etapie odcinanym)
			double t2; //czas redukowanego kawalka (relatywny)
			printf("trzeci stopien");

			t2 = pose_list_iterator->t - ((pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) / pose_list_iterator->a_r[i]);

			s1 = s - (pose_list_iterator->v_k[i] * (pose_list_iterator->t - t2)
			   + 0.5 * pose_list_iterator->a_r[i] *(pose_list_iterator->t - t2) * (pose_list_iterator->t - t2));

			if (pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i] * t2 * t2 //liczba pod pierwiastkiem mniejsza od 0, zabezpieczenie
				- 4 * pose_list_iterator->a_r[i] * (pose_list_iterator->v_k[i] * t2 - s1) < 0) {
				vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
				return;
			}

			t1 = (pose_list_iterator->a_r[i] * t2
					- (sqrt(pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i] * t2 * t2
					- 4 * pose_list_iterator->a_r[i] * (pose_list_iterator->v_k[i] * t2 - s1))) )
					/ (2 * pose_list_iterator->a_r[i]);

			if ((pose_list_iterator->v_k[i] - pose_list_iterator->a_r[i] * t1) < 0) {//ujemna predkosc ruchu, zabezpieczenie
				vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
				return;
			}

			pose_list_iterator->v_r[i] = pose_list_iterator->v_k[i] - pose_list_iterator->a_r[i] * t1;
			pose_list_iterator->s_przysp[i] = 0.5 * pose_list_iterator->a_r[i] * t1 * t1 + pose_list_iterator->v_r[i] * t1
											+ pose_list_iterator->v_k[i] * (pose_list_iterator->t - t2)
											+ 0.5 * pose_list_iterator->a_r[i] * (pose_list_iterator->t - t2) * (pose_list_iterator->t - t2);
			pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * (t2 - 2 * t1);

			return;
		}

		pose_list_iterator->a_r[i] = a;
		pose_list_iterator->v_r[i] = pose_list_iterator->v_k[i];
		pose_list_iterator->s_przysp[i] = pose_list_iterator->v_r[i] * (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])/pose_list_iterator->a_r[i]
		                                + 0.5 * (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) * (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])
		                                /pose_list_iterator->a_r[i];
		pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * (pose_list_iterator->t
									  - (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])/pose_list_iterator->a_r[i]);

		return;
	}

	pose_list_iterator->a_r[i] = a;
	pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i];
	pose_list_iterator->s_przysp[i] = 0;
	pose_list_iterator->s_jedn[i] = pose_list_iterator->v_p[i] * (pose_list_iterator->t - (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) / pose_list_iterator->a_r[i]);
}

void smooth2::vp_reduction(std::list<ecp_smooth2_taught_in_pose>::iterator pose_list_iterator, int i, double s, double t) {
	printf("v_p redukcja\n");
	//TODO sprawdzic czy to wyczerpuje wszystkie przypadki... chyba tak
	double v_r; //zmiana ruchu na jednostajny

	v_r = s/t;

	if (v_r < pose_list_iterator->v_k[i] && v_r < pose_list_iterator->v_p[i]) {
		printf("rekurencja!");
		pose_list_iterator->v_r[i] = v_r;
		rec = true;
		calculate();
		return;
	} else if (v_r < pose_list_iterator->v_p[i] && v_r > pose_list_iterator->v_k[i]) {
		pose_list_iterator->v_p[i] = v_r;
		reduction_model_4(pose_list_iterator, i, s);
		rec = true;
		calculate();
		return;
	} else if (v_r > pose_list_iterator->v_p[i] && v_r < pose_list_iterator->v_k[i]) {
		pose_list_iterator->v_k[i] = v_r;
		reduction_model_2(pose_list_iterator, i, s);
		rec = true;
		calculate();
		return;
	} else {
		sr_ecp_msg.message("Are you insane?! That trajectory can not be calculated!");
		throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}
}

void smooth2::vk_reduction(std::list<ecp_smooth2_taught_in_pose>::iterator pose_list_iterator, int i, double s, double t) {
	printf("v_k redukcja\n");
	double a;
	double v_k;

	a = (2 * (s - (pose_list_iterator->v_p[i] * t))) / (t * t);

	if (a < 0) {
		printf("v_k stopien 2\n");
		a = (-2 * s + 2 * t * pose_list_iterator->v_p[i]) / (t * t);
		v_k = -a * t + pose_list_iterator->v_p[i];

		if (a > pose_list_iterator->a_r[i] || v_k < 0) {
			vp_reduction(pose_list_iterator, i, s, t);
			return;
		}

		pose_list_iterator->v_k[i] = v_k;
		pose_list_iterator->a_r[i] = a;
		pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i];
		pose_list_iterator->s_przysp[i] = 0;
		pose_list_iterator->s_jedn[i] = 0;
		return;
	}

	pose_list_iterator->v_k[i] = a * t + pose_list_iterator->v_p[i];
	pose_list_iterator->a_r[i] = a;
	pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i];
	pose_list_iterator->s_przysp[i] = 0;
	pose_list_iterator->s_jedn[i] = 0;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
