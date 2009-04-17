// -------------------------------------------------------------------------
//                            ecp_g_smooth2.cc
//            Effector Control Process (lib::ECP) - smooth2 generator
// generator powstal na podstawie generatora smooth, glowna zmiana jest
// rezygnacja z podawanie predkosci poczatkowej i koncowej w kazdym ruchu
// autor: Rafal Tulwin
// Ostatnia modyfikacja: 2009
// -------------------------------------------------------------------------


//TODO porobic zabezpieczenia w funkcjach obslugujacych pose list
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

/*void ecp_smooth2_generator::generate_next_coords (void)
{
	next_position[1] -= 0.002;
    //funkcja obliczajaca polozenie w danym makrokroku

    int i;
    double tk=10*STEP;

    for(i=0; i<MAX_SERVOS_NR; i++)
    {
        if(node_counter<przysp[i])
        { //pierwszy etap
            if(v_p[i]<=v_r[i])
            { //przyspieszanie w pierwszym etapie

                next_position[i] = start_position[i] +
                                   k[i]*(node_counter*v_p[i]*tk + node_counter*node_counter*a_r[i]*tk*tk/2);
            }
            else
            { //hamowanie w pierwszym etapie

                next_position[i] = start_position[i] +
                                   k[i]*((node_counter*tk*v_p[i]) -
                                         (node_counter*node_counter*tk*tk*a_r[i]/2));
            }
        }
        else if(node_counter<=jedn[i])
        { // drugi etap - ruch jednostajny
            next_position[i] = start_position[i] +
                               k[i]*(s_przysp[i] + (node_counter*tk - fabs(v_r[i]-v_p[i])/a_r[i])*v_r[i]);
        }
        else if(node_counter<=td.interpolation_node_no)
        { //trzeci etap
            if(v_k[i]<=v_r[i])
            { //hamowanie w trzecim etapie

                next_position[i] = start_position[i] +
                                   k[i]*(s_przysp[i] + s_jedn[i] +
                                         (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*v_r[i] -
                                         (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*(node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*a_r[i]/2);
            }
            else
            { //przyspieszanie w trzecim etapie

                next_position[i] = start_position[i] +
                                   k[i]*(s_przysp[i] + s_jedn[i] +
                                         (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*v_r[i] +
                                         (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*(node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*a_r[i]/2);
            }
        }
    }
}*/

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
	first_coordinate=true;
	trajectory_generated = false;
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

// -------------------------------------------------------get all previously saved elements from actual iterator
void smooth2::get_pose(void) {
    //int i;

    td.arm_type = pose_list_iterator->arm_type;
/*    for(i=0; i<MAX_SERVOS_NR; i++)
    {
        v[i]=pose_list_iterator->v[i];
        a[i]=pose_list_iterator->a[i];
        final_position[i]=pose_list_iterator->coordinates[i];
    }*/
//	 if(type==2)
//					for(i=0; i<MAX_SERVOS_NR; i++)
//						final_position[i]+=start_position[i];

}
// -------------------------------------------------------
void smooth2::set_pose (lib::POSE_SPECIFICATION ps, double vv[MAX_SERVOS_NR], double aa[MAX_SERVOS_NR], double c[MAX_SERVOS_NR]) {
    pose_list_iterator->arm_type = ps;
    memcpy(pose_list_iterator->coordinates, c, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->v, vv, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->a, aa, MAX_SERVOS_NR*sizeof(double));
}

bool smooth2::is_pose_list_element(void) {
    // sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
    if ( pose_list_iterator != pose_list->end())
    {
        return true;
    }
    else
    {
        return false;
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

void smooth2::generate_cords() {

	double coordinate[MAX_SERVOS_NR];

	coordinate[0] = 0.800;
	coordinate[1] = -0.399;
	coordinate[2] = 0.435;
	coordinate[3] = -1.136;
	coordinate[4] = 1.390;
	coordinate[5] = 2.351;
	coordinate[6] = 0.074;

	coordinate_list->push_back(coordinates(coordinate));

	int i;

	for (i = 0; i < 9; i++) {
		coordinate[0] = 0.810;
		coordinate[1] = -0.399;
		coordinate[2] = 0.435;
		coordinate[3] = -1.136;
		coordinate[4] = 1.390;
		coordinate[5] = 2.351;
		coordinate[6] = 0.074;

		coordinate_list->push_back(coordinates(coordinate));
	}

	/*initiate_coordinate_list();

	for (i = 0; i < 9; i++) {
		for (int j = 0; j < 7; j++)
			printf("%f\n", coordinate_list_iterator->coordinate[j]);
		coordinate_list_iterator++;
	}*/
	printf("\ngenerate_cords\n");
}

//set necessary instructions, and other data for preparing the robot
bool smooth2::first_step() { //wywolywane tylko raz w calej trajektorii

    initiate_pose_list();
    get_pose();
    //td.arm_type = pose_list_iterator->arm_type;

    first_interval=true;
    switch ( td.arm_type )
    {

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

    //printf("first_step wywolanie\n");

    return true;
} // end: bool ecp_smooth2_generator::first_step ( )

bool smooth2::next_step () {

    int i; //licznik petli
    //int j;
    //double tk = 10*STEP; //czas jednego makrokroku

    printf("\n============================================================\n poczatek \n ============================================================= \n");

    if (!trajectory_generated) {
    	calculate(); //wypelnienie pozostalych zmiennych w liscie pose_list
    	generate_cords(); //wypelnienie listy coordinate_list (lista kolejnych pozycji w makrokrokach)
    	trajectory_generated = true;
    	initiate_pose_list(); //ustawienie iteratora pose_list na poczatek
    	initiate_coordinate_list(); //ustawienie iteratora coordinate_list na poczatek
    	pose_list_iterator->interpolation_node_no = 5;
    	/*next_pose_list_ptr();
    	pose_list_iterator->interpolation_node_no = 6;
    	next_pose_list_ptr();
    	pose_list_iterator->interpolation_node_no = 4;
    	initiate_pose_list();
    	next_position[0] = 0.849;
    	next_position[1] = -0.288;
    	next_position[2] = 0.305;
    	next_position[3] = -1.136;
    	next_position[4] = 1.390;
    	next_position[5] = 2.351;
    	next_position[6] = 0.074;*/
    	printf("trajektoria wygenerowana\n");
    }

    //printf("node count: %d\n", node_counter);
    //printf("interpolation_node_no: %d\n", pose_list_iterator->interpolation_node_no);

    if (node_counter-1 == pose_list_iterator->interpolation_node_no) {//czy poprzedni makrokrok byl ostatnim

    	//printf("po ostatnim makrokroku ruchu\n");

    	if(is_last_list_element()) { //ostatni punkt (koniec listy pozycji pose_list)
    		//printf("ostatni element pose_list\n");
    		return false;
    	} else {//lista pozycji pose_list nie jest skonczona wiec idziemy do nastepnego punktu
    		//printf("wywolany else (nastepna pozycja pose_list)\n");
			node_counter = 0; //ustawienie makrokroku na 1
		    next_pose_list_ptr();
		    td.interpolation_node_no = pose_list_iterator->interpolation_node_no;
		}
    } else {
    	//printf("kolejny makrokrok\n");
    	the_robot->EDP_data.instruction_type = lib::SET; //ustawienie parametrow ruchu w edp_data
    	the_robot->EDP_data.get_type = NOTHING_DV; //ponizej w caseach jest dalsze ustawianie
    	the_robot->EDP_data.get_arm_type = lib::INVALID_END_EFFECTOR;

    	switch ( td.arm_type ) {//TODO dodac wiecej opcji wspolrzednych
    		case lib::XYZ_EULER_ZYZ:
    		//	printf("wejscie do eulera\n");
    			the_robot->EDP_data.instruction_type = lib::SET; //dalsze ustawianie parametrow ruchu w edp
    			the_robot->EDP_data.set_type = ARM_DV; // ARM
    			the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
    			the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    			the_robot->EDP_data.next_interpolation_type = lib::MIM;
    			the_robot->EDP_data.motion_steps = td.internode_step_no;
    			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

    			if(node_counter < pose_list_iterator->interpolation_node_no) {//jezeli makrokrok nie jest ostatnim makrokrokiem w ruchu

    			    printf("normalny makrokrok\n");
    				//printf("%f\n", next_position[1]);
    			    //generate_next_coords(); //obliczanie nastepnych wspolrzednych makrokroku
    			    //printf("\n %f\n", coordinate_list_iterator->coordinate[0]);
    			    for (i = 0; i < 6; i++) {//zapisanie nastepnego polazenia (makrokroku) do robota
    			        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = coordinate_list_iterator->coordinate[i];
    			    }

					coordinate_list_iterator++;

    			    //PROBA Z CHWYTAKIEM

    			    /*if(pose_list_iterator->v_grip*node_counter < pose_list_iterator->coordinates[6]) {
    			        the_robot->EDP_data.next_gripper_coordinate = pose_list_iterator->v_grip*node_counter;
    			    } else {
    			        the_robot->EDP_data.next_gripper_coordinate = pose_list_iterator->coordinates[6];
    			    }*/
    			    the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;//TODO to jest tymczasowe wiec trzeba poprawic
    			} else {
    			//	printf("ostatni makrokrok ruchu\n");
    			//OSTATNI PUNKT
    				for (i = 0; i < 6; i++) //ostatni makrokrok, przypisujemy final_position (coordinates)
    			        the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = pose_list_iterator->coordinates[i];

    				//the_robot->EDP_data.next_gripper_coordinate = pose_list_iterator->coordinates[6];
    				the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;//TODO tutaj musi byc cos innego
    			}

    	//		printf("koncowka w switchu\n");
    			break;

    		default:
    			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    	}// end:switch
  //  	printf("koncowka ifa \n");
    }// end: if

    return true;

} // end: bool ecp_smooth2_generator::next_step ( )

void smooth2::calculate(void) {

	double s[MAX_SERVOS_NR];
	double s_temp1[MAX_SERVOS_NR], s_temp2[MAX_SERVOS_NR];
	double t_temp1, t_temp2;
    double t[MAX_SERVOS_NR];
    double t_max; //nadluzszy czas ruchu w jednej osi w jednym ruchu
    //double v_r[MAX_SERVOS_NR], a_r[MAX_SERVOS_NR];
    int i;
    double tk = 10 * STEP; //czas jednego makrokroku

    double v_r_next[MAX_SERVOS_NR];

    double temp;//generalny temp do wszystkiego
    //double v_r_prev[MAX_SERVOS_NR];

    initiate_pose_list();


    td.internode_step_no = 10;
    td.value_in_step_no = td.internode_step_no - 2;

    for (int j = 0; j < pose_list_length(); j++) {
    	printf("petla listy pozycji\n");
		td.arm_type = pose_list_iterator->arm_type;

		if (first_interval) {//pierwszy makrokrok ruchu
			//wywoluje sie tylko raz, przy pierwzszym makrokroku pierwszego ruchu, musi byc rozroznione, gdyz tutaj v_p jest = 0 a pozycja jest
			// ...odczytywana z ramienia
			//get_pose();//pobranie parametrow nowego ruchu (zapisanie zmiennych v, a i final_position(coordinates))

			switch (td.arm_type) {

			case lib::XYZ_EULER_ZYZ:
				printf("euler w first_interval\n");
				for (i = 0; i < 6; i++) {
					pose_list_iterator->start_position[i] = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];//pierwsze przypisanie start_position
				}

				pose_list_iterator->start_position[6] = the_robot->EDP_data.current_gripper_coordinate;
				pose_list_iterator->start_position[7] = 0.0;//TODO sprawdzic czy tutaj ma byc 0

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					pose_list_iterator->v_r[i] = v_max_zyz[i] * pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_zyz[i] * pose_list_iterator->a[i];
					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if(pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
						pose_list_iterator->k[i]=-1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
			        }
			        else {
			        	pose_list_iterator->k[i]=1;
			        }
				}

				break;

			default:
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);

			} // end: switch

			first_interval = false;

		} else { // end: if(first_interval)
			switch (td.arm_type) {

			case lib::XYZ_EULER_ZYZ:
				for (i = 0; i < 6; i++) {
					temp = pose_list_iterator->coordinates[i];
					next_pose_list_ptr();
					pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					next_pose_list_ptr();
					pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
					prev_pose_list_ptr();					//predkoscia nowego ruchu
				}

				next_pose_list_ptr(); //inkrementacja iteratora listy pose_list

				pose_list_iterator->start_position[6] = the_robot->EDP_data.current_gripper_coordinate;
				pose_list_iterator->start_position[7] = 0.0;//TODO sprawdzic czy tutaj ma byc 0

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					printf("predkosci (max i zadane): %f\t %f\n", v_max_zyz[i], pose_list_iterator->v[i]);
					printf("przyspieszenia (max i zadane): %f\t %f\n", a_max_zyz[i], pose_list_iterator->a[i]);
					pose_list_iterator->v_r[i] = v_max_zyz[i] * pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_zyz[i] * pose_list_iterator->a[i];
				}

				break;

			default:
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
		}

		for(i=0;i<MAX_SERVOS_NR;i++) {//zapisanie coordinate_delta
			td.coordinate_delta[i] = pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i];

		}

		for (i = 0; i < MAX_SERVOS_NR; i++) {//zapisanie v_r_next
			if (is_last_list_element()) {
				printf("ostatni ruch\n");
				v_r_next[i] = 0.0;
				pose_list_iterator->v_k[i] = 0; //to nie jest potrzebne tak naprawde...
			} else {
				temp = pose_list_iterator->k[i];
				next_pose_list_ptr();

				if(pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i] < 0) {//nadpisanie k dla nastepnego ruchu
					pose_list_iterator->k[i]=-1;
		        }
		        else {
		        	pose_list_iterator->k[i]=1;
		        }

				if (pose_list_iterator->k[i] != temp) {
					v_r_next[i] = 0;
				} else {
					v_r_next[i] = v_max_zyz[i] * pose_list_iterator->v[i];
				}
				printf("ruch nieostatni, nastepna predkosc: %f\n", v_max_zyz[i] * pose_list_iterator->v[i]);
				prev_pose_list_ptr();
			}
		}
		// ==================================== poczatek oliczen ========================================

		t_max = 0;

		//pose_list_iterator->start_position[1] = -0.288; //tymczasowa opcja (powrot do pozycji synchro)

		//obliczenie drogi dla kazdej osi
		for (i = 0; i < MAX_SERVOS_NR; i++) {
			s[i]=fabs(pose_list_iterator->coordinates[i] - pose_list_iterator->start_position[i]);
			printf("%f\n", s[i]);
		}
		//sprawdzanie czy droga w etapach przyspieszenia i hamowania nie jest wieksza niz droga calego ruchu

		//warunki na modele ruchu dla wszystkich osi
		for (i = 0; i < MAX_SERVOS_NR; i++) { //petla w ktorej obliczany jest czas dla kazdej osi i sprawdzane jest czy da sie wykonac ruch w zalozonych etapach

			if (s[i] < 0.001) {//najmniejsza wykrywalna droga
				printf("droga 0 %d\n", i);
				continue;
			}

			if (pose_list_iterator->v_p[i] < pose_list_iterator->v_r[i] && v_r_next[i] < pose_list_iterator->v_r[i]) { //pierwszy model
				printf("pierwszy model (droga) %d\n", i);
				s_temp1[i] = 0.5 * pose_list_iterator->a_r[i] * ((pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i]) * ((pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i]);
				s_temp2[i] = 0.5 * pose_list_iterator->a_r[i] * ((pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i]) * ((pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i]);

				printf("s_temp1: %f\ts_temp2: %f\n", s_temp1[i], s_temp2[i]);

				if (s_temp1[i] + s_temp2[i] > s[i]) {
					printf("redukcja predkosci w osi %d\n", i);
					//nadpisanie v_r (ruch bedzie 2 etapowy)
					//obliczenie czasu
				} else {//droga przyspieszenia i opoznienia nie przekracza drogi ruchu
					t_temp1 = (pose_list_iterator->v_r[i] - pose_list_iterator->v_p[i])/pose_list_iterator->a_r[i];
					t_temp2 = (pose_list_iterator->v_r[i] - v_r_next[i])/pose_list_iterator->a_r[i];

					printf("t_temp1: %f\tt_temp2: %f\n", t_temp1, t_temp2);

					t[i] = t_temp1 + t_temp2 + (s[i] - (s_temp1[i] + s_temp2[i]))/pose_list_iterator->v_r[i];

					printf("czas\t\tv_r\t\tv_r_next\ta_r\t\tv_p\n");
					printf("%f\t%f\t%f\t%f\t%f\n", t[i], pose_list_iterator->v_r[i], v_r_next[i], pose_list_iterator->a_r[i], pose_list_iterator->v_p[i]);

				}

				pose_list_iterator->v_k[i] = v_r_next[i];

				if (t[i] > t_max) {//napisanie najdluzszego czasu
					t_max = t[i];
				}

            } else if (pose_list_iterator->v_p[i] < pose_list_iterator->v_r[i] && (v_r_next[i] > pose_list_iterator->v_r[i] || eq(v_r_next[i], pose_list_iterator->v_r[i]))) { // drugi model

			} else if (eq(pose_list_iterator->v_p[i], pose_list_iterator->v_r[i]) && (v_r_next[i] > pose_list_iterator->v_r[i] || eq(v_r_next[i], pose_list_iterator->v_r[i]))) { //trzeci model

			} else if (eq(pose_list_iterator->v_p[i], pose_list_iterator->v_r[i]) && v_r_next[i] < pose_list_iterator->v_r[i]) { //czwarty model
				printf("czwarty model\n");



				printf("czas\t\tv_r\t\tv_r_next\ta_r\t\tv_p\n");
									printf("%f\t%f\t%f\t%f\t%f\n", t[i], pose_list_iterator->v_r[i], v_r_next[i], pose_list_iterator->a_r[i], pose_list_iterator->v_p[i]);

			} else {
				printf("\n ten przypadek nigdy nie moze wystapic\n");
			}
		}

	    //kwantyzacja czasu
	    if(ceil(t_max/tk)*tk != t_max)
	    {
	        t_max = ceil(t_max/tk);
	        t_max = t_max*tk;
	    }

		pose_list_iterator->interpolation_node_no = (int)round(t_max / tk);

		for (i = 0; i < MAX_SERVOS_NR; i++) {

			if (t[i] < t_max) {//redukcja predkosci w danej osi
				//redukcja predkosci i przypisanie jedn, przysp
			} else {
				//zapisanie jedn przysp

			}
		}
    }
    /*for(i=0; i<MAX_SERVOS_NR; i++)
    {
        // Obliczenie drog dla wszystkich osi
        s[i]=fabs(final_position[i]-start_position[i]);
        // kierunek ruchu
        if(final_position[i]-start_position[i] < 0)
        {
            k[i]=-1;
        }
        else
        {
            k[i]=1;
        }

        if(eq(s[i],0.0))
        {
            t=0;
        }
        else if(
            ((v_r[i]>=v_p[i]) && (v[i]>=v_k[i]) && (((2*v_p[i]*(v_r[i]-v_p[i]) + 2*v_r[i]*(v_r[i]-v_k[i])
                                                    +((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) - ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]<v_p[i]) && (v[i]>=v_k[i]) && (((2*v_p[i]*(v_p[i]-v_r[i]) + 2*v_r[i]*(v_r[i]-v_k[i])
                                                    -((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) - ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]>=v_p[i]) && (v[i]<v_k[i]) && (((2*v_p[i]*(v_r[i]-v_p[i]) + 2*v_r[i]*(v_k[i] - v_r[i])
                                                    +((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) + ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]<v_p[i]) && (v[i]<v_k[i]) && (((2*v_p[i]*(v_p[i]-v_r[i]) + 2*v_r[i]*(v_k[i] - v_r[i])
                                                    -((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) + ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
        )

            // zwykly ruch w 3 etapach
        {
            if(debug)
            {
                printf("%d - 3 etapy\n", i);
            }
            // Obliczenie najdluzszego czasu
            if((v_r[i]>=v_p[i]) && (v_r[i]>=v_k[i]))
            {// pierwszy model ruchu - przyspieszanie / hamowanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) - (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) + (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]>=v_p[i]) && (v_r[i]<v_k[i]))
            {// drugi model ruchu - przyspieszanie / przyspieszanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) - (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) - (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]<v_p[i]) && (v_r[i]>=v_k[i]))
            {// trzeci model ruchu - hamowanie / hamowanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) + (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) + (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]<v_p[i]) && (v_r[i]<v_k[i]))
            {// czwarty model ruchu - hamowanie / przyspieszanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) + (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) - (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }

        } //end - obliczanie czasu w 3 etapach
        else //jesli droga jest krotsza niz osiagnieta przy przyspieszaniu i hamowaniu przy zadanych a i v
        {
            switch(i)
            {
            case 0:
                sr_ecp_msg.message("Redukcja predkosci w osi 0");
                break;
            case 1:
                sr_ecp_msg.message("Redukcja predkosci w osi 1");
                break;
            case 2:
                sr_ecp_msg.message("Redukcja predkosci w osi 2");
                break;
            case 3:
                sr_ecp_msg.message("Redukcja predkosci w osi 3");
                break;
            case 4:
                sr_ecp_msg.message("Redukcja predkosci w osi 4");
                break;
            case 5:
                sr_ecp_msg.message("Redukcja predkosci w osi 5");
                break;
            case 6:
                sr_ecp_msg.message("Redukcja predkosci w osi 6");
                break;
            case 7:
                sr_ecp_msg.message("Redukcja predkosci w osi 7");
                break;
            }
            if(debug)
            {
                printf("%d - 2 etapy\n", i);
            }
            v_1=sqrt(v_p[i]*v_p[i]/2 + v_k[i]*v_k[i]/2 + s[i]*a_r[i]);
            if(v_1>=v_p[i] && v_1>=v_k[i])
            {
                v_r[i]=v_1;
            }
            else
            {
                v_1=(v_p[i] + v_k[i])/2;
                if((v_1<v_p[i] && v_1>=v_k[i]) || (v_1>=v_p[i] && v_1<v_k[i]))
                    v_r[i]=v_1;
                else
                {
                    v_1=sqrt(v_p[i]*v_p[i]/2 + v_k[i]*v_k[i]/2 - s[i]*a_r[i]);
                    if(v_1<v_p[i] && v_1<v_k[i])
                    {
                        v_r[i] = v_1;
                    }
                    else
                    {
                        printf("Blad w obliczaniu predkosci w 2 etapach!!\n");
                        throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
                    }

                }
            }

            t = (fabs(v_r[i] - v_p[i]) + fabs(v_r[i] - v_k[i]))/a_r[i];
        }
        if(t>t_max)
            t_max=t;
    }

    //kwantyzacja czasu
    if(ceil(t_max/tk)*tk != t_max)
    {
        t_max = ceil(t_max/tk);
        t_max = t_max*tk;
    }

    // Obliczenie predkosci dla wszystkich osi
    for(i=0; i<MAX_SERVOS_NR; i++) {
        if(eq(s[i],0.0)) {
            v_r[i]=0;
            printf("brak drogi\n");
        }
        else
        {//pierszy model ruchu - przyspieszanie / hamowanie
            v_1=(v_p[i]+v_k[i]+a_r[i]*t_max)/2 + sqrt(-4*v_p[i]*v_p[i]
                    -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]+8*v_p[i]*a_r[i]*t_max+8*v_k[i]
                    *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                    -16*a_r[i]*s[i])/4;

            v_2=(v_p[i]+v_k[i]+a_r[i]*t_max)/2 - sqrt(-4*v_p[i]*v_p[i]
                    -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]+8*v_p[i]*a_r[i]*t_max+8*v_k[i]
                    *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                    -16*a_r[i]*s[i])/4;
            printf("po pierwszym v1 = %lf, v2 = %lf\n", v_1, v_2);
            if((v_1>=0)&&(v_1<=v_r[i])&&(v_1>=v_p[i])&&(v_1>=v_k[i])) {
                v_r[i]=v_1;
                printf("1 - przyspieszanie hamowanie\n");
            }
            else if((v_2>=0)&&(v_2<=v_r[i])&&(v_2>=v_p[i])&&(v_2>=v_k[i])) {
                v_r[i]=v_2;
                printf("1 - przyspieszanie hamowanie\n");
            }
            else
            {//drugi model ruchu - przyspieszanie / przyspieszanie
                v_1 = (v_k[i]*v_k[i] - 2*a_r[i]*s[i] - v_p[i]*v_p[i])/(2*(v_k[i]-v_p[i]-a_r[i]*t_max));
                printf("po drugim v1 = %lf, v2 = %lf\n", v_1, v_2);
                if((v_1>=0)&&(v_1<=v_r[i])&&(v_1>=v_p[i])&&(v_1<v_k[i])) {
                    v_r[i]=v_1;
                    printf("2 - przyspieszanie przyspieszanie\n");
                }
                else
                {//trzeci model ruchu - hamowanie / hamowanie
                    v_1 = (-2*a_r[i]*s[i] + v_p[i]*v_p[i] - v_k[i]*v_k[i])/(2*(v_p[i]-v_k[i]-a_r[i]*t_max));
                    printf("po trzecim v1 = %lf, v2 = %lf\n", v_1, v_2);
                    if((v_1>=0)&&(v_1<=v_r[i])&&(v_1<v_p[i])&&(v_1>=v_k[i])) {
                        v_r[i]=v_1;
                        printf("3 - hamowanie hamowanie\n");
                    }
                    else
                    {//czwarty model ruchu - hamowanie / przyspieszanie
                        v_1=(v_p[i]+v_k[i]-a_r[i]*t_max)/2 + sqrt(-4*v_p[i]*v_p[i]
                                -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]-8*v_p[i]*a_r[i]*t_max-8*v_k[i]
                                *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                                +16*a_r[i]*s[i])/4;

                        v_2=(v_p[i]+v_k[i]-a_r[i]*t_max)/2 - sqrt(-4*v_p[i]*v_p[i]
                                -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]-8*v_p[i]*a_r[i]*t_max-8*v_k[i]
                                *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                                +16*a_r[i]*s[i])/4;

                        if((v_1>=0)&&(v_1<=v_r[i])&&(v_1<v_p[i])&&(v_1<v_k[i])) {
                            v_r[i]=v_1;
                            printf("4 - hamowanie przyspieszanie\n");
                        }
                        else if((v_2>=0)&&(v_2<=v_r[i])&&(v_2<v_p[i])&&(v_2<v_k[i])) {
                            v_r[i]=v_2;
                            printf("4 - hamowanie przyspieszanie\n");
                        }
                        else
                        {//blad - brak rozwiazania
                            printf("blad! nie da sie obliczyc predkosci (%d)\n", i);
                            //throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
                            v_r[i]=v_1;
                        }
                    }
                }
            }

            if(eq(s[i],0.0))
                v_r[i]=0;
            printf("koniec v1 = %lf, v2 = %lf\n\n\n", v_1, v_2);
        }
    }*/

    // Wypelnienie struktury td
    /*td.interpolation_node_no = (int)round(t_max / tk);
    td.internode_step_no = 10;
    td.value_in_step_no = td.internode_step_no - 2;

    for(i=0;i<MAX_SERVOS_NR;i++)
        td.coordinate_delta[i] = final_position[i]-
                                 start_position[i];
   /*if(debug)
    {
        printf("makrokroki: %d, mikrokroki: %d, czas kroku: %f, step: %f\n", td.interpolation_node_no, td.internode_step_no, tk, STEP);
        printf("t: %f\n", t_max);
        printf("v: %f, %f, %f, %f, %f, %f, %f, %f\n", v_r[0], v_r[1], v_r[2], v_r[3], v_r[4], v_r[5], v_r[6], v_r[7]);
    }*/

    /*for(i=0;i<MAX_SERVOS_NR;i++)
    {
        przysp[i]=fabs((v_r[i]-v_p[i])/(a_r[i]*tk));
        jedn[i]=(t_max-(fabs(v_r[i]-v_k[i])/a_r[i]))/tk;

        if(v_r[i]>=v_p[i])
        {
            s_przysp[i]=(2*v_p[i]*(v_r[i]-v_p[i]) + (v_r[i] - v_p[i])*(v_r[i] - v_p[i]))/(2*a_r[i]);
        }
        else
        {
            s_przysp[i]=(2*v_p[i]*(v_p[i] - v_r[i]) - (v_r[i] - v_p[i])*(v_r[i] - v_p[i]))/(2*a_r[i]);
        }

        s_jedn[i]= (t_max - (fabs(v_r[i] - v_p[i]) + fabs(v_r[i] - v_k[i]))/a_r[i])*v_r[i];
    }

    v_grip =	(final_position[i]/td.interpolation_node_no);
    if(v_grip<v_grip_min)
        v_grip=v_grip_min;*/


}//end - calculate

/*bool ecp_smooth2_generator::next_step ()
{
    int i;
    double tk=10*STEP; //czas jednego makrokroku


    // ---------------------------------   FIRST INTERVAL    ---------------------------------------
    if ( first_interval )//pierwszy makrokrok ruchu
    {//wywoluje sie tylko raz, przy pierwzszym makrokroku pierwszego ruchu, pozniej te obliczenia robione sa w nastepnym warunku "if"
                                                  // ...po ostatnim makrokroku ruchu

        t_max=0;
        get_pose();//pobranie parametrow nowego ruchu (zapisanie zmiennych v, a i final_position(coordinates))

        // Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
        // aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
        // to dopiero execute_motion po wyjsciu z first_step.

        switch ( td.arm_type )
        {
        case lib::MOTOR:
            for(i=0;i<MAX_SERVOS_NR;i++){
                start_position[i]=the_robot->EDP_data.current_motor_arm_coordinates[i];
					 if(type==2)
						final_position[i]+=start_position[i];
				}
            for(i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_motor[i]*v[i];
                a_r[i]=a_max_motor[i]*a[i];
                v_p[i]=0;
                v_k[i]=0;
            }
            calculate();
            break;

        case lib::JOINT:
            for(i=0;i<MAX_SERVOS_NR;i++)
                start_position[i]=the_robot->EDP_data.current_joint_arm_coordinates[i];
            for(i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_joint[i]*v[i];
                a_r[i]=a_max_joint[i]*a[i];
                v_p[i]=0;
                v_k[i]=0;
            }
            calculate();
            break;

        case lib::XYZ_EULER_ZYZ:
            for(i=0;i<6;i++)
                start_position[i]=the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];//pierwsze przypisanie start_position
            start_position[6]=the_robot->EDP_data.current_gripper_coordinate;
            start_position[7]=0.0;
            for(i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_zyz[i]*v[i];
                a_r[i]=a_max_zyz[i]*a[i];
                v_p[i]=0;
                v_k[i]=0;
            }
            printf("w first interval\n");
            calculate();
            break;
        case lib::XYZ_ANGLE_AXIS:
            for(i=0;i<6;i++)
                start_position[i]=the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
            start_position[6]=the_robot->EDP_data.current_gripper_coordinate;
            start_position[7]=0.0;
            for(i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_aa[i]*v[i];
                a_r[i]=a_max_aa[i]*a[i];
                v_p[i]=0;
                v_k[i]=0;
            }
            calculate();
            break;
        default:
            throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
        } // end:switch

        first_interval = false;
    }	// end:if FIRST INTERVAL
    // -------------------------------------------------------------------------------------------

    // Kontakt z MP
    if (node_counter-1 == td.interpolation_node_no) //czy poprzedni makrokrok byl ostatnim
    { // Koniec odcinka
        if(is_last_list_element())	//ostatni punkt (koniec listy pozycji pose_list)
        {
        	fflush(stdout);
        	printf("po ostatnim elemencie trakejtorii\n");
            return false;
        }
        else //lista pozycji nie jest skonczona wiec idziemy do nastepnego punktu
        {
            t_max=0;
            for(i=0; i<MAX_SERVOS_NR; i++){
					if(type==1) //wybor wzgledny/bezwzgledny
						start_position[i]=pose_list_iterator->coordinates[i]; //przypisanie wartosci pozycji koncowej poprzedniego...
					if(type==2)// ...ruchu jako wartosci poczatkowej nowego ruchu
						start_position[i]+=pose_list_iterator->coordinates[i];
				}
            next_pose_list_ptr(); //przesuniecie iteratora na nastepna pozycje

            get_pose(); //zapisanie v, a, final_position (= coordinates) z listy pozycji

				if(type==2)
					for(i=0; i<MAX_SERVOS_NR; i++)
						final_position[i]+=start_position[i];


            // Przepisanie danych z EDP_MASTER do obrazu robota


            switch ( td.arm_type )
            {
            case lib::MOTOR:
                for(i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_motor[i]*v[i];
                    a_r[i]=a_max_motor[i]*a[i];
                    v_p[i]=v_k[i];
                    v_k[i]=0;
                }
                calculate();
                break;

            case lib::JOINT:
                for(i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_joint[i]*v[i];
                    a_r[i]=a_max_joint[i]*a[i];
                    v_p[i]=v_k[i];
                    v_k[i]=0;
                }
                calculate();
                break;

            case lib::XYZ_EULER_ZYZ:
                for(i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_zyz[i]*v[i];
                    a_r[i]=a_max_zyz[i]*a[i];
                    v_p[i]=v_k[i];
                    v_k[i]=0;
                }
                printf("ostatni makrokrok ruchu\n");
                fflush(stdout);
                calculate();
                break;
            case lib::XYZ_ANGLE_AXIS:
                for(i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_aa[i]*v[i];
                    a_r[i]=a_max_aa[i]*a[i];
                    v_p[i]=v_k[i];
                    v_k[i]=0;
                }
                calculate();
                break;
            default:
                throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
            } // end:switch
      		 node_counter=1; //ustawienie makrokroku na 1

        }
    } //koniec: nastepny punkt trajektorii


    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

    the_robot->EDP_data.instruction_type = lib::SET; //ustawienie parametrow ruchu w edp_data
    the_robot->EDP_data.get_type = NOTHING_DV; //ponizej w caseach jest dalsze ustawianie
    the_robot->EDP_data.get_arm_type = lib::INVALID_END_EFFECTOR;


    switch ( td.arm_type )
    {
    case lib::MOTOR:
        the_robot->EDP_data.instruction_type = lib::SET;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = lib::MOTOR;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_motor_arm_coordinates[i] = next_position[i];

            //PROBA Z CHWYTAKIEM

            if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK)
                i=8;
            else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT)
                i=7;

            if(v_grip*node_counter < final_position[i])
            {
                the_robot->EDP_data.next_motor_arm_coordinates[i] = v_grip*node_counter;
            }
            else
            {
                the_robot->EDP_data.next_motor_arm_coordinates[i] = final_position[i];
            }
        }
        else
        {
            //OSTATNI PUNKT
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_motor_arm_coordinates[i] = final_position[i];
        }
        break;

    case lib::JOINT:
            the_robot->EDP_data.instruction_type = lib::SET;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = lib::JOINT;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
         the_robot->EDP_data.next_interpolation_type = lib::MIM;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_joint_arm_coordinates[i] = next_position[i];

            //PROBA Z CHWYTAKIEM

            if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK)
                i=8;
            else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT)
                i=7;

            if(v_grip*node_counter < final_position[i])
            {
                the_robot->EDP_data.next_joint_arm_coordinates[i] = v_grip*node_counter;
            }
            else
            {
                the_robot->EDP_data.next_joint_arm_coordinates[i] = final_position[i];
            }
        }
        else
        {
            //OSTATNI PUNKT
            generate_next_coords();
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_joint_arm_coordinates[i] = final_position[i];
        }
        break;

    case lib::XYZ_EULER_ZYZ:
            the_robot->EDP_data.instruction_type = lib::SET; //dalsze ustawianie parametrow ruchu w edp
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
         the_robot->EDP_data.next_interpolation_type = lib::MIM;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;



        if(node_counter < td.interpolation_node_no) //jezeli makrokrok nie jest ostatnim makrokrokiem w ruchu
        {
        	//printf("wywolanie generate_next_cords\n");
            generate_next_coords(); //obliczanie nastepnych wspolrzednych makrokroku
            for (i=0; i<6; i++)//zapisanie nastepnego polazenia (makrokroku) do robota
                the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = next_position[i];

            //PROBA Z CHWYTAKIEM

            if(v_grip*node_counter < final_position[6])
            {
                the_robot->EDP_data.next_gripper_coordinate = v_grip*node_counter;
            }
            else
            {
                the_robot->EDP_data.next_gripper_coordinate = final_position[6];
            }
        }
        else
        {
            //OSTATNI PUNKT
            for (i=0; i<6; i++) //ostatni makrokrok, przypisujemy final_position
                the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = final_position[i];
            the_robot->EDP_data.next_gripper_coordinate = final_position[6];
        }

        break;

    case lib::XYZ_ANGLE_AXIS:
            the_robot->EDP_data.instruction_type = lib::SET;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
         the_robot->EDP_data.next_interpolation_type = lib::MIM;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = next_position[i];

            the_robot->EDP_data.next_gripper_coordinate = next_position[6];
        }
        else
        {
            //OSTATNI PUNKT
            for (i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = final_position[i];
            the_robot->EDP_data.next_gripper_coordinate = final_position[6];
        }
        break;
    default:
            throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    }// end:switch


    return true;

}*/ // end: BOOLEAN ecp_smooth2_generator::next_step ( )

/*void ecp_smooth2_generator::calculate(void)
{
    double s[MAX_SERVOS_NR];
    double t;
    double v_1, v_2;
    int i, tmp;
    double tk=10*STEP; //czas jednego makrokroku

    for(i=0; i<MAX_SERVOS_NR; i++)
    {
        // Obliczenie drog dla wszystkich osi
        s[i]=fabs(final_position[i]-start_position[i]);
        // kierunek ruchu
        if(final_position[i]-start_position[i] < 0)
        {
            k[i]=-1;
        }
        else
        {
            k[i]=1;
        }

        if(eq(s[i],0.0))
        {
            t=0;
        }
        else if(
            ((v_r[i]>=v_p[i]) && (v[i]>=v_k[i]) && (((2*v_p[i]*(v_r[i]-v_p[i]) + 2*v_r[i]*(v_r[i]-v_k[i])
                                                    +((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) - ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]<v_p[i]) && (v[i]>=v_k[i]) && (((2*v_p[i]*(v_p[i]-v_r[i]) + 2*v_r[i]*(v_r[i]-v_k[i])
                                                    -((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) - ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]>=v_p[i]) && (v[i]<v_k[i]) && (((2*v_p[i]*(v_r[i]-v_p[i]) + 2*v_r[i]*(v_k[i] - v_r[i])
                                                    +((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) + ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]<v_p[i]) && (v[i]<v_k[i]) && (((2*v_p[i]*(v_p[i]-v_r[i]) + 2*v_r[i]*(v_k[i] - v_r[i])
                                                    -((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) + ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
        )

            // zwykly ruch w 3 etapach
        {
            if(debug)
            {
                printf("%d - 3 etapy\n", i);
            }
            // Obliczenie najdluzszego czasu
            if((v_r[i]>=v_p[i]) && (v_r[i]>=v_k[i]))
            {// pierwszy model ruchu - przyspieszanie / hamowanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) - (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) + (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]>=v_p[i]) && (v_r[i]<v_k[i]))
            {// drugi model ruchu - przyspieszanie / przyspieszanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) - (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) - (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]<v_p[i]) && (v_r[i]>=v_k[i]))
            {// trzeci model ruchu - hamowanie / hamowanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) + (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) + (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]<v_p[i]) && (v_r[i]<v_k[i]))
            {// czwarty model ruchu - hamowanie / przyspieszanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) + (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) - (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }

        } //end - obliczanie czasu w 3 etapach
        else //jesli droga jest krotsza niz osiagnieta przy przyspieszaniu i hamowaniu przy zadanych a i v
        {
            switch(i)
            {
            case 0:
                sr_ecp_msg.message("Redukcja predkosci w osi 0");
                break;
            case 1:
                sr_ecp_msg.message("Redukcja predkosci w osi 1");
                break;
            case 2:
                sr_ecp_msg.message("Redukcja predkosci w osi 2");
                break;
            case 3:
                sr_ecp_msg.message("Redukcja predkosci w osi 3");
                break;
            case 4:
                sr_ecp_msg.message("Redukcja predkosci w osi 4");
                break;
            case 5:
                sr_ecp_msg.message("Redukcja predkosci w osi 5");
                break;
            case 6:
                sr_ecp_msg.message("Redukcja predkosci w osi 6");
                break;
            case 7:
                sr_ecp_msg.message("Redukcja predkosci w osi 7");
                break;
            }
            if(debug)
            {
                printf("%d - 2 etapy\n", i);
            }
            v_1=sqrt(v_p[i]*v_p[i]/2 + v_k[i]*v_k[i]/2 + s[i]*a_r[i]);
            if(v_1>=v_p[i] && v_1>=v_k[i])
            {
                v_r[i]=v_1;
            }
            else
            {
                v_1=(v_p[i] + v_k[i])/2;
                if((v_1<v_p[i] && v_1>=v_k[i]) || (v_1>=v_p[i] && v_1<v_k[i]))
                    v_r[i]=v_1;
                else
                {
                    v_1=sqrt(v_p[i]*v_p[i]/2 + v_k[i]*v_k[i]/2 - s[i]*a_r[i]);
                    if(v_1<v_p[i] && v_1<v_k[i])
                    {
                        v_r[i] = v_1;
                    }
                    else
                    {
                        printf("Blad w obliczaniu predkosci w 2 etapach!!\n");
                        throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
                    }

                }
            }

            t = (fabs(v_r[i] - v_p[i]) + fabs(v_r[i] - v_k[i]))/a_r[i];
        }
        if(t>t_max)
            t_max=t;
    }

    //kwantyzacja czasu
    if(ceil(t_max/tk)*tk != t_max)
    {
        t_max = ceil(t_max/tk);
        t_max = t_max*tk;
    }

    // Obliczenie predkosci dla wszystkich osi
    for(i=0; i<MAX_SERVOS_NR; i++) {
        if(eq(s[i],0.0)) {
            v_r[i]=0;
            printf("brak drogi\n");
        }
        else
        {//pierszy model ruchu - przyspieszanie / hamowanie
            v_1=(v_p[i]+v_k[i]+a_r[i]*t_max)/2 + sqrt(-4*v_p[i]*v_p[i]
                    -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]+8*v_p[i]*a_r[i]*t_max+8*v_k[i]
                    *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                    -16*a_r[i]*s[i])/4;

            v_2=(v_p[i]+v_k[i]+a_r[i]*t_max)/2 - sqrt(-4*v_p[i]*v_p[i]
                    -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]+8*v_p[i]*a_r[i]*t_max+8*v_k[i]
                    *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                    -16*a_r[i]*s[i])/4;
            printf("po pierwszym v1 = %lf, v2 = %lf\n", v_1, v_2);
            if((v_1>=0)&&(v_1<=v_r[i])&&(v_1>=v_p[i])&&(v_1>=v_k[i])) {
                v_r[i]=v_1;
                printf("1 - przyspieszanie hamowanie\n");
            }
            else if((v_2>=0)&&(v_2<=v_r[i])&&(v_2>=v_p[i])&&(v_2>=v_k[i])) {
                v_r[i]=v_2;
                printf("1 - przyspieszanie hamowanie\n");
            }
            else
            {//drugi model ruchu - przyspieszanie / przyspieszanie
                v_1 = (v_k[i]*v_k[i] - 2*a_r[i]*s[i] - v_p[i]*v_p[i])/(2*(v_k[i]-v_p[i]-a_r[i]*t_max));
                printf("po drugim v1 = %lf, v2 = %lf\n", v_1, v_2);
                if((v_1>=0)&&(v_1<=v_r[i])&&(v_1>=v_p[i])&&(v_1<v_k[i])) {
                    v_r[i]=v_1;
                    printf("2 - przyspieszanie przyspieszanie\n");
                }
                else
                {//trzeci model ruchu - hamowanie / hamowanie
                    v_1 = (-2*a_r[i]*s[i] + v_p[i]*v_p[i] - v_k[i]*v_k[i])/(2*(v_p[i]-v_k[i]-a_r[i]*t_max));
                    printf("po trzecim v1 = %lf, v2 = %lf\n", v_1, v_2);
                    if((v_1>=0)&&(v_1<=v_r[i])&&(v_1<v_p[i])&&(v_1>=v_k[i])) {
                        v_r[i]=v_1;
                        printf("3 - hamowanie hamowanie\n");
                    }
                    else
                    {//czwarty model ruchu - hamowanie / przyspieszanie
                        v_1=(v_p[i]+v_k[i]-a_r[i]*t_max)/2 + sqrt(-4*v_p[i]*v_p[i]
                                -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]-8*v_p[i]*a_r[i]*t_max-8*v_k[i]
                                *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                                +16*a_r[i]*s[i])/4;

                        v_2=(v_p[i]+v_k[i]-a_r[i]*t_max)/2 - sqrt(-4*v_p[i]*v_p[i]
                                -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]-8*v_p[i]*a_r[i]*t_max-8*v_k[i]
                                *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                                +16*a_r[i]*s[i])/4;

                        if((v_1>=0)&&(v_1<=v_r[i])&&(v_1<v_p[i])&&(v_1<v_k[i])) {
                            v_r[i]=v_1;
                            printf("4 - hamowanie przyspieszanie\n");
                        }
                        else if((v_2>=0)&&(v_2<=v_r[i])&&(v_2<v_p[i])&&(v_2<v_k[i])) {
                            v_r[i]=v_2;
                            printf("4 - hamowanie przyspieszanie\n");
                        }
                        else
                        {//blad - brak rozwiazania
                            printf("blad! nie da sie obliczyc predkosci (%d)\n", i);
                            //throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
                            v_r[i]=v_1;
                        }
                    }
                }
            }

            if(eq(s[i],0.0))
                v_r[i]=0;
            printf("koniec v1 = %lf, v2 = %lf\n\n\n", v_1, v_2);
        }
    }

    // Wypelnienie struktury td
    td.interpolation_node_no = (int)round(t_max / tk);
    td.internode_step_no = 10;
    td.value_in_step_no = td.internode_step_no - 2;

    for(i=0;i<MAX_SERVOS_NR;i++)
        td.coordinate_delta[i] = final_position[i]-
                                 start_position[i];
   if(debug)
    {
        printf("makrokroki: %d, mikrokroki: %d, czas kroku: %f, step: %f\n", td.interpolation_node_no, td.internode_step_no, tk, STEP);
        printf("t: %f\n", t_max);
        printf("v: %f, %f, %f, %f, %f, %f, %f, %f\n", v_r[0], v_r[1], v_r[2], v_r[3], v_r[4], v_r[5], v_r[6], v_r[7]);
    }

    for(i=0;i<MAX_SERVOS_NR;i++)
    {
        przysp[i]=fabs((v_r[i]-v_p[i])/(a_r[i]*tk));
        jedn[i]=(t_max-(fabs(v_r[i]-v_k[i])/a_r[i]))/tk;

        if(v_r[i]>=v_p[i])
        {
            s_przysp[i]=(2*v_p[i]*(v_r[i]-v_p[i]) + (v_r[i] - v_p[i])*(v_r[i] - v_p[i]))/(2*a_r[i]);
        }
        else
        {
            s_przysp[i]=(2*v_p[i]*(v_p[i] - v_r[i]) - (v_r[i] - v_p[i])*(v_r[i] - v_p[i]))/(2*a_r[i]);
        }

        s_jedn[i]= (t_max - (fabs(v_r[i] - v_p[i]) + fabs(v_r[i] - v_k[i]))/a_r[i])*v_r[i];
    }

    v_grip =	(final_position[i]/td.interpolation_node_no);
    if(v_grip<v_grip_min)
        v_grip=v_grip_min;


} *///end - calculate

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
