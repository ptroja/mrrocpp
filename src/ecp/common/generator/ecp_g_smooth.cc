// -------------------------------------------------------------------------
//                            generator/ecp_g_smooth.cc
//            Effector Control Process (lib::ECP) - smooth generator
// Generator powstal na podstawie generatora smooth, glowna zmiana jest
// rezygnacja z podawanie predkosci poczatkowej i koncowej w kazdym ruchu.
// W przeciwienstwie do smootha generator smooth nie dopuszcza nigdy do przekroczenia
// maksymalnej podanej predkosci ruchu i przyspieszenia dla danej osi.
// Generator z zalozenia jest w stanie policzyc kazda trajektorie.
// autor: Rafal Tulwin
// Ostatnia modyfikacja: 2009
// -------------------------------------------------------------------------

#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

void smooth::set_relative(void) {
	type = lib::RELATIVE;
}

void smooth::set_absolute(void) {
	type = lib::ABSOLUTE;
}

bool smooth::eq(double a, double b) {
	const double EPS = 0.0001;
	const double& diff = a - b;
	return diff < EPS && diff > -EPS;
}

void smooth::load_trajectory_from_xml(ecp_mp::common::Trajectory &trajectory) {
	bool first_time = true;
	int numOfPoses = trajectory.getNumberOfPoses();
	trajectory.showTime();

	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
	pose_list = trajectory.getPoses();
	pose_list_iterator = pose_list.end();
}

void smooth::set_pose_from_xml(xmlNode *stateNode, bool &first_time) {
	char *dataLine, *value;
	uint64_t number_of_poses; // Liczba zapamietanych pozycji
	lib::ECP_POSE_SPECIFICATION ps; // Rodzaj wspolrzednych
	double v[MAX_SERVOS_NR];
	double a[MAX_SERVOS_NR]; // Wczytane wspolrzedne
	double coordinates[MAX_SERVOS_NR]; // Wczytane wspolrzedne

	xmlNode *cchild_node, *ccchild_node;
	xmlChar *coordinateType, *numOfPoses;
	xmlChar *xmlDataLine;

	coordinateType = xmlGetProp(stateNode, (const xmlChar *) "coordinateType");
	ps = lib::returnProperPS((char *) coordinateType);
	numOfPoses = xmlGetProp(stateNode, (const xmlChar *) "numOfPoses");
	number_of_poses = (uint64_t) atoi((const char *) numOfPoses);
	for (cchild_node = stateNode->children; cchild_node != NULL; cchild_node
			= cchild_node->next) {
		if (cchild_node->type == XML_ELEMENT_NODE && !xmlStrcmp(
				cchild_node->name, (const xmlChar *) "Pose")) {
			for (ccchild_node = cchild_node->children; ccchild_node != NULL; ccchild_node
					= ccchild_node->next) {
				if (ccchild_node->type == XML_ELEMENT_NODE && !xmlStrcmp(
						ccchild_node->name, (const xmlChar *) "Velocity")) {
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					lib::setValuesInArray(v, (const char *) xmlDataLine);
					xmlFree(xmlDataLine);
				}
				if (ccchild_node->type == XML_ELEMENT_NODE && !xmlStrcmp(
						ccchild_node->name, (const xmlChar *) "Accelerations")) {
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					lib::setValuesInArray(a, (const char *) xmlDataLine);
					xmlFree(xmlDataLine);
				}
				if (ccchild_node->type == XML_ELEMENT_NODE && !xmlStrcmp(
						ccchild_node->name, (const xmlChar *) "Coordinates")) {
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					lib::setValuesInArray(coordinates,
							(const char *) xmlDataLine);
					xmlFree(xmlDataLine);
				}
			}
			if (first_time) {
				// Tworzymy glowe listy
				first_time = false;
				create_pose_list_head(ps, v, a, coordinates);
				//printf("Pose list head: %d, %f, %f, %f, %f, %f\n", ps, vp[0], vk[0], v[0], a[0], coordinates[0]);
			} else {
				// Wstaw do listy nowa pozycje
				insert_pose_list_element(ps, v, a, coordinates);
				//printf("Pose list element: %d, %f, %f, %f, %f, %f\n", ps, vp[0], vk[0], v[0], a[0], coordinates[0]);
			}
		}
	}
	xmlFree(coordinateType);
	xmlFree(numOfPoses);
}

void smooth::load_trajectory_from_xml(const char* fileName,
		const char* nodeName) {
	// Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,

	bool first_time = true; // Znacznik

	xmlDocPtr doc = xmlParseFile(fileName);
	xmlXIncludeProcess(doc);
	if (doc == NULL) {
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	xmlNodePtr root = xmlDocGetRootElement(doc);
	if (!root || !root->name) {
		xmlFreeDoc(doc);
		throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje

	for (xmlNodePtr cur_node = root->children; cur_node != NULL; cur_node
			= cur_node->next) {
		if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name,
				(const xmlChar *) "SubTask")) {
			for (xmlNodePtr subTaskNode = cur_node->children; subTaskNode
					!= NULL; subTaskNode = subTaskNode->next) {
				if (subTaskNode->type == XML_ELEMENT_NODE && !xmlStrcmp(
						subTaskNode->name, (const xmlChar *) "State")) {
					xmlChar * stateID = xmlGetProp(subTaskNode,
							(const xmlChar *) "id");
					if (stateID && !strcmp((const char *) stateID, nodeName)) {
						for (xmlNodePtr child_node = subTaskNode->children; child_node
								!= NULL; child_node = child_node->next) {
							if (child_node->type == XML_ELEMENT_NODE
									&& !xmlStrcmp(child_node->name,
											(const xmlChar *) "Trajectory")) {
								set_pose_from_xml(child_node, first_time);
							}
						}
					}
					xmlFree(stateID);
				}
			}
		}
		if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name,
				(const xmlChar *) "State")) {
			xmlChar * stateID = xmlGetProp(cur_node, (const xmlChar *) "id");
			if (stateID && !strcmp((const char *) stateID, nodeName)) {
				for (xmlNodePtr child_node = cur_node->children; child_node
						!= NULL; child_node = child_node->next) {
					if (child_node->type == XML_ELEMENT_NODE && !xmlStrcmp(
							child_node->name, (const xmlChar *) "Trajectory")) {
						set_pose_from_xml(child_node, first_time);
					}
				}
			}
			xmlFree(stateID);
		}
	}
	xmlFreeDoc(doc);
	xmlCleanupParser();
}

void smooth::load_file_with_path(const char* file_name) {

	reset();

	sr_ecp_msg.message(file_name);
	// Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,
	//printf("load file with path\n");
	//flushall();
	char coordinate_type[80]; // Opis wspolrzednych: "MOTOR", "JOINT", ...
	lib::ECP_POSE_SPECIFICATION ps; // Rodzaj wspolrzednych
	uint64_t e; // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba zapamietanych pozycji
	uint64_t i, j; // Liczniki petli
	bool first_time = true; // Znacznik
	int extra_info;
	double v[MAX_SERVOS_NR];
	double a[MAX_SERVOS_NR]; // Wczytane wspolrzedne
	double coordinates[MAX_SERVOS_NR]; // Wczytane wspolrzedne

	std::ifstream from_file(file_name); // otworz plik do odczytu
	if (!from_file.good()) {
		perror(file_name);
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	if (!(from_file >> coordinate_type)) {
		throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	// Usuwanie spacji i tabulacji
	i = 0;
	j = 0;
	while (coordinate_type[i] == ' ' || coordinate_type[i] == '\t')
		i++;
	while (coordinate_type[i] != ' ' && coordinate_type[i] != '\t'
			&& coordinate_type[i] != '\n' && coordinate_type[i] != '\r'
			&& coordinate_type[j] != '\0') {
		coordinate_type[j] = toupper(coordinate_type[i]);
		i++;
		j++;
	}
	coordinate_type[j] = '\0';

	if (!strcmp(coordinate_type, "MOTOR")) {
		ps = lib::ECP_MOTOR;
	} else if (!strcmp(coordinate_type, "JOINT")) {
		ps = lib::ECP_JOINT;
	} else if (!strcmp(coordinate_type, "XYZ_EULER_ZYZ")) {
		ps = lib::ECP_XYZ_EULER_ZYZ;
	} else if (!strcmp(coordinate_type, "XYZ_ANGLE_AXIS")) {
		ps = lib::ECP_XYZ_ANGLE_AXIS;
	}

	else {
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
	}

	// printf("po coord type %d\n", ps);
	if (!(from_file >> number_of_poses)) {
		throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	// printf("po number of poses %d\n", number_of_poses);
	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
	// printf("po flush pose list\n");


	for (i = 0; i < number_of_poses; i++) {
		//printf("w petli\n");
		// printf("po vk\n");

		for (j = 0; j < MAX_SERVOS_NR; j++) {
			if (!(from_file >> v[j])) { // Zabezpieczenie przed danymi nienumerycznymi
				throw generator::ECP_error(lib::NON_FATAL_ERROR,
						READ_FILE_ERROR);
			}
		}

		// printf("po v\n");
		for (j = 0; j < MAX_SERVOS_NR; j++) {
			if (!(from_file >> a[j])) { // Zabezpieczenie przed danymi nienumerycznymi
				throw generator::ECP_error(lib::NON_FATAL_ERROR,
						READ_FILE_ERROR);
			}
		}

		// printf("po a\n");
		for (j = 0; j < MAX_SERVOS_NR; j++) {
			if (!(from_file >> coordinates[j])) { // Zabezpieczenie przed danymi nienumerycznymi
				throw generator::ECP_error(lib::NON_FATAL_ERROR,
						READ_FILE_ERROR);
			}
		}

		if (first_time) {
			// Tworzymy glowe listy
			first_time = false;
			create_pose_list_head(ps, v, a, coordinates);
		} else {
			// Wstaw do listy nowa pozycje
			insert_pose_list_element(ps, v, a, coordinates);
			// printf("Pose list element: %d, %f, %f, %f, %f\n", ps, vp[0], vk[0], v[0], a[0]);
		}
	} // end: for
} // end: load_file_with_path()

//jesli w ponizszych metodach podamy reset jako true lista pozycji zostanie wyczyszczona, jesli jako false pozycja zostanie dodana do listy bez jej czyszczenia
void smooth::load_coordinates(lib::ECP_POSE_SPECIFICATION ps,
		double coordinates[MAX_SERVOS_NR], bool reset) {

	double v[MAX_SERVOS_NR] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.5 };
	double a[MAX_SERVOS_NR] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.5 };

	if (reset == true) {
		flush_pose_list();
	}

	if (pose_list_length() == 0) {
		create_pose_list_head(ps, v, a, coordinates);
	} else { // Wstaw do listy nowa pozycje
		insert_pose_list_element(ps, v, a, coordinates);
	}
}

void smooth::load_coordinates(lib::ECP_POSE_SPECIFICATION ps,
		double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR],
		double coordinates[MAX_SERVOS_NR], bool reset) {

	if (reset == true) {
		flush_pose_list();
	}

	if (pose_list_length() == 0) {
		create_pose_list_head(ps, v, a, coordinates);
	} else { // Wstaw do listy nowa pozycje
		insert_pose_list_element(ps, v, a, coordinates);
	}
}

void smooth::load_coordinates(lib::ECP_POSE_SPECIFICATION ps, double cor0,
		double cor1, double cor2, double cor3, double cor4, double cor5,
		double cor6, double cor7, bool reset) {

	double v[MAX_SERVOS_NR] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };
	double
			a[MAX_SERVOS_NR] = { 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05 };
	double coordinates[MAX_SERVOS_NR];

	if (reset == true) {
		flush_pose_list();
	}

	coordinates[0] = cor0;
	coordinates[1] = cor1;
	coordinates[2] = cor2;
	coordinates[3] = cor3;
	coordinates[4] = cor4;
	coordinates[5] = cor5;
	coordinates[6] = cor6;
	coordinates[7] = cor7;

	if (pose_list_length() == 0) {
		create_pose_list_head(ps, v, a, coordinates);
	} else { // Wstaw do listy nowa pozycje
		insert_pose_list_element(ps, v, a, coordinates);
	}
}

void smooth::load_coordinates(lib::ECP_POSE_SPECIFICATION ps,
		double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double cor0,
		double cor1, double cor2, double cor3, double cor4, double cor5,
		double cor6, double cor7, bool reset) {

	double coordinates[MAX_SERVOS_NR];

	if (reset == true) {
		flush_pose_list();
	}

	coordinates[0] = cor0;
	coordinates[1] = cor1;
	coordinates[2] = cor2;
	coordinates[3] = cor3;
	coordinates[4] = cor4;
	coordinates[5] = cor5;
	coordinates[6] = cor6;
	coordinates[7] = cor7;

	if (pose_list_length() == 0) {
		create_pose_list_head(ps, v, a, coordinates);
	} else { // Wstaw do listy nowa pozycje
		insert_pose_list_element(ps, v, a, coordinates);
	}
}

void smooth::load_xyz_angle_axis(double coordinates[MAX_SERVOS_NR], bool reset) {
	load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, coordinates, reset);
}
void smooth::load_xyz_angle_axis(double v[MAX_SERVOS_NR],
		double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR], bool reset) {
	load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, coordinates, reset);
}
void smooth::load_xyz_angle_axis(double cor0, double cor1, double cor2,
		double cor3, double cor4, double cor5, double cor6, double cor7,
		bool reset) {
	load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, cor0, cor1, cor2, cor3, cor4,
			cor5, cor6, cor7, reset);
}
void smooth::load_xyz_angle_axis(double v[MAX_SERVOS_NR],
		double a[MAX_SERVOS_NR], double cor0, double cor1, double cor2,
		double cor3, double cor4, double cor5, double cor6, double cor7,
		bool reset) {
	load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, cor0, cor1, cor2, cor3,
			cor4, cor5, cor6, cor7, reset);
}

void smooth::load_xyz_euler_zyz(double coordinates[MAX_SERVOS_NR], bool reset) {
	load_coordinates(lib::ECP_XYZ_EULER_ZYZ, coordinates, reset);
}
void smooth::load_xyz_euler_zyz(double v[MAX_SERVOS_NR],
		double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR], bool reset) {
	load_coordinates(lib::ECP_XYZ_EULER_ZYZ, v, a, coordinates, reset);
}
void smooth::load_xyz_euler_zyz(double cor0, double cor1, double cor2,
		double cor3, double cor4, double cor5, double cor6, double cor7,
		bool reset) {
	load_coordinates(lib::ECP_XYZ_EULER_ZYZ, cor0, cor1, cor2, cor3, cor4,
			cor5, cor6, cor7, reset);
}
void smooth::load_xyz_euler_zyz(double v[MAX_SERVOS_NR],
		double a[MAX_SERVOS_NR], double cor0, double cor1, double cor2,
		double cor3, double cor4, double cor5, double cor6, double cor7,
		bool reset) {
	load_coordinates(lib::ECP_XYZ_EULER_ZYZ, v, a, cor0, cor1, cor2, cor3,
			cor4, cor5, cor6, cor7, reset);
}

void smooth::load_joint(double coordinates[MAX_SERVOS_NR], bool reset) {
	load_coordinates(lib::ECP_JOINT, coordinates, reset);
}
void smooth::load_joint(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR],
		double coordinates[MAX_SERVOS_NR], bool reset) {
	load_coordinates(lib::ECP_JOINT, v, a, coordinates, reset);
}
void smooth::load_joint(double cor0, double cor1, double cor2, double cor3,
		double cor4, double cor5, double cor6, double cor7, bool reset) {
	load_coordinates(lib::ECP_JOINT, cor0, cor1, cor2, cor3, cor4, cor5, cor6,
			cor7, reset);
}
void smooth::load_joint(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR],
		double cor0, double cor1, double cor2, double cor3, double cor4,
		double cor5, double cor6, double cor7, bool reset) {
	load_coordinates(lib::ECP_JOINT, v, a, cor0, cor1, cor2, cor3, cor4, cor5,
			cor6, cor7, reset);
}

void smooth::load_motor(double coordinates[MAX_SERVOS_NR], bool reset) {
	load_coordinates(lib::ECP_MOTOR, coordinates, reset);
}
void smooth::load_motor(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR],
		double coordinates[MAX_SERVOS_NR], bool reset) {
	load_coordinates(lib::ECP_MOTOR, v, a, coordinates, reset);
}
void smooth::load_motor(double cor0, double cor1, double cor2, double cor3,
		double cor4, double cor5, double cor6, double cor7, bool reset) {
	load_coordinates(lib::ECP_MOTOR, cor0, cor1, cor2, cor3, cor4, cor5, cor6,
			cor7, reset);
}
void smooth::load_motor(double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR],
		double cor0, double cor1, double cor2, double cor3, double cor4,
		double cor5, double cor6, double cor7, bool reset) {
	load_coordinates(lib::ECP_MOTOR, v, a, cor0, cor1, cor2, cor3, cor4, cor5,
			cor6, cor7, reset);
}

void smooth::reset() {
	flush_pose_list();//TODO sprawdzic czy to jest potrzebne
	flush_coordinate_list();//TODO sprawdzic czy to jest potrzebne
	//first_coordinate = true;
	first_interval = true;
	trajectory_generated = false;
	trajectory_calculated = false;
}

void smooth::flush_pose_list(void) {
	pose_list.clear();
	//first_coordinate=true;
}

// -------------------------------------------------------return iterator to beginning of the list
void smooth::initiate_pose_list(void) {
	pose_list_iterator = pose_list.begin();
}
// -------------------------------------------------------move pose_list iterator to the next position
void smooth::next_pose_list_ptr(void) {
	if (pose_list_iterator != pose_list.end()) {
		pose_list_iterator++;
	}
}

void smooth::prev_pose_list_ptr(void) {
	if (pose_list_iterator != pose_list.begin()) {
		pose_list_iterator--;
	}
}

bool smooth::is_last_list_element(void) {
	// sprawdza czy aktualnie wskazywany element listy ma nastepnik
	// jesli <> nulla
	if (pose_list_iterator != pose_list.end()) {
		if ((++pose_list_iterator) != pose_list.end()) {
			--pose_list_iterator;
			return false;
		} else {
			--pose_list_iterator;
			return true;
		}
	}
	return false;
}

void smooth::create_pose_list_head(lib::ECP_POSE_SPECIFICATION ps,
		double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR],
		double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(ecp_mp::common::smooth_trajectory_pose(ps, coordinates,
			v, a));
	pose_list_iterator = pose_list.begin();
}

void smooth::insert_pose_list_element(lib::ECP_POSE_SPECIFICATION ps,
		double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR],
		double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(ecp_mp::common::smooth_trajectory_pose(ps, coordinates,
			v, a));
	pose_list_iterator++;
}

int smooth::pose_list_length(void) {
	return pose_list.size();
}

void smooth::initiate_coordinate_list(void) {
	coordinate_list_iterator = coordinate_list.begin();
}

void smooth::flush_coordinate_list(void) {
	coordinate_list.clear();
}

void smooth::load_a_v_min(const char* file_name) {
	std::ifstream from_file(file_name); // otworz plik do odczytu

	if (!from_file.good()) {
		perror(file_name);
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	if (!(from_file >> v_grip_min_zyz)) { // Zabezpieczenie przed danymi nienumerycznymi
		throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	if (!(from_file >> v_grip_min_aa)) { // Zabezpieczenie przed danymi nienumerycznymi
		throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	if (!(from_file >> v_grip_min_joint)) { // Zabezpieczenie przed danymi nienumerycznymi
		throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	if (!(from_file >> v_grip_min_motor)) { // Zabezpieczenie przed danymi nienumerycznymi
		throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}
} // end: bool load_a_v_min()

void smooth::load_a_v_max(const char* file_name) {
	std::ifstream from_file(file_name); // otworz plik do odczytu

	if (!from_file.good()) {
		perror(file_name);
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	for (int j = 0; j < MAX_SERVOS_NR; j++) {
		if (!(from_file >> v_max_motor[j])) { // Zabezpieczenie przed danymi nienumerycznymi
			throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
	}
	for (int j = 0; j < MAX_SERVOS_NR; j++) {
		if (!(from_file >> a_max_motor[j])) { // Zabezpieczenie przed danymi nienumerycznymi
			throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
	}

	for (int j = 0; j < MAX_SERVOS_NR; j++) {
		if (!(from_file >> v_max_joint[j])) { // Zabezpieczenie przed danymi nienumerycznymi
			throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
	}
	for (int j = 0; j < MAX_SERVOS_NR; j++) {
		if (!(from_file >> a_max_joint[j])) { // Zabezpieczenie przed danymi nienumerycznymi
			throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
	}

	for (int j = 0; j < MAX_SERVOS_NR; j++) {
		if (!(from_file >> v_max_zyz[j])) { // Zabezpieczenie przed danymi nienumerycznymi
			throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
	}
	for (int j = 0; j < MAX_SERVOS_NR; j++) {
		if (!(from_file >> a_max_zyz[j])) { // Zabezpieczenie przed danymi nienumerycznymi
			throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
	}

	for (int j = 0; j < MAX_SERVOS_NR; j++) {
		if (!(from_file >> v_max_aa[j])) { // Zabezpieczenie przed danymi nienumerycznymi
			throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
	}
	for (int j = 0; j < MAX_SERVOS_NR; j++) {
		if (!(from_file >> a_max_aa[j])) { // Zabezpieczenie przed danymi nienumerycznymi
			throw generator::ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
	}
} // end: bool load_a_v_max()

smooth::smooth(common::task::task& _ecp_task, bool _is_synchronised) :
	delta(_ecp_task), debug(false)//,first_coordinate(true)
{
	pose_list = std::list<ecp_mp::common::smooth_trajectory_pose>();
	coordinate_list = std::list<coordinates>();

	trajectory_generated = false;
	trajectory_calculated = false;

	distance_eps = 0.00001;

	std::string max_path(ecp_t.mrrocpp_network_path);
	max_path += "src/ecp_mp/a_v_max.txt";

	std::string min_path(ecp_t.mrrocpp_network_path);
	min_path += "src/ecp_mp/v_min_gripp.txt";

	load_a_v_max(max_path.c_str());
	load_a_v_min(min_path.c_str());

	//is_synchronised = _is_synchronised;
	type = lib::ABSOLUTE;
} // end : konstruktor

/*void smooth::calculate_absolute_positions() {
 if (type == lib::ABSOLUTE) {//dodatkowe zabezpieczenie
 return;
 }

 double actual_coordinates[MAX_SERVOS_NR];
 bool first_coordinate = true;
 int gripp;

 initiate_pose_list();

 for (int j = 0; j < pose_list_length(); j++) {
 switch ( td.arm_type ) {
 case lib::MOTOR:

 if(the_robot->robot_name == lib::ROBOT_IRP6OT_M) {
 gripp=7;
 } else if(the_robot->robot_name == lib::ROBOT_IRP6P_M) {
 gripp=6;
 }

 for (int i = 0; i < gripp; i++) {
 if (first_coordinate == true) {
 actual_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
 }
 pose_list_iterator->coordinates[i] += actual_coordinates[i];
 actual_coordinates[i] = pose_list_iterator->coordinates[i];
 }

 if (first_coordinate == true) {
 actual_coordinates[gripp] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
 }
 pose_list_iterator->coordinates[gripp] += actual_coordinates[gripp];
 actual_coordinates[gripp] = pose_list_iterator->coordinates[gripp];

 first_coordinate = false;
 break;
 case lib::JOINT:

 if(the_robot->robot_name == lib::ROBOT_IRP6OT_M) {
 gripp=7;
 } else if(the_robot->robot_name == lib::ROBOT_IRP6P_M) {
 gripp=6;
 }

 for (int i = 0; i < gripp; i++) {
 if (first_coordinate == true) {
 actual_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
 }
 pose_list_iterator->coordinates[i] += actual_coordinates[i];
 actual_coordinates[i] = pose_list_iterator->coordinates[i];
 }

 if (first_coordinate == true) {
 actual_coordinates[gripp] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
 }
 pose_list_iterator->coordinates[gripp] += actual_coordinates[gripp];
 actual_coordinates[gripp] = pose_list_iterator->coordinates[gripp];

 first_coordinate = false;
 break;
 case lib::XYZ_EULER_ZYZ:
 for (int i = 0; i < 6; i++) {
 if (first_coordinate == true) {
 actual_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
 }
 pose_list_iterator->coordinates[i] += actual_coordinates[i];
 actual_coordinates[i] = pose_list_iterator->coordinates[i];
 }

 if (first_coordinate == true) {
 actual_coordinates[6] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
 }
 pose_list_iterator->coordinates[6] += actual_coordinates[6];
 actual_coordinates[6] = pose_list_iterator->coordinates[6];

 first_coordinate = false;
 break;
 case lib::XYZ_ANGLE_AXIS:
 for (int i = 0; i < 6; i++) {
 if (first_coordinate == true) {
 actual_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
 }
 pose_list_iterator->coordinates[i] += actual_coordinates[i];
 actual_coordinates[i] = pose_list_iterator->coordinates[i];
 }

 if (first_coordinate == true) {
 actual_coordinates[6] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
 }
 pose_list_iterator->coordinates[6] += actual_coordinates[6];
 actual_coordinates[6] = pose_list_iterator->coordinates[6];

 first_coordinate = false;
 break;
 default:
 throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
 } // end : switch ( td.arm_type )
 pose_list_iterator++;
 }
 }*/

double smooth::generate_next_coords(int node_counter,
		int interpolation_node_no, double start_position, double v_p,
		double v_r, double v_k, double a_r, int k, double przysp, double jedn,
		double s_przysp, double s_jedn) {

	//funkcja obliczajaca polozenie w danym makrokroku

	double next_position;

	double tk = 10 * STEP;

	//printf("przysp: %f\t jedn: %f\n", przysp, jedn);

	if (node_counter < przysp + 1) { //pierwszy etap
		if (v_p <= v_r) { //przyspieszanie w pierwszym etapie
			//printf("start pos: %f\t node counter: %d\n", start_position, node_counter);
			//printf(" przysp1 %d ", node_counter);
			if (type == lib::ABSOLUTE) {//tryb absolute

				if (przysp > (node_counter - 1) && przysp < (node_counter)) {//przyspieszanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku

						next_position = start_position + k * (przysp * v_p * tk
								+ przysp * przysp * a_r * tk * tk / 2 + jedn
								* tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk - a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp
									- jedn) * tk - tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn
									- przysp) * a_r / 2);
						} else {//przyspiesznie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk + a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp
									- jedn) * tk + tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn
									- przysp) * a_r / 2);
						}
					} else {//przyspieszenie + poczatek jednostajnego
						//printf("przyspieszanie wchodzi na jednostajny, macrostep liczony od 1: %d\n", node_counter);
						//next_position = start_position + k * ((node_counter-1)*v_p*tk + (node_counter-1)*(node_counter-1)*a_r*tk*tk/2);
						//next_position += k * (v_p*tk + a_r*tk*tk/2 + (v_p+a_r*(node_counter-1) *tk)) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp);
						next_position = start_position + k * (v_p
								* node_counter * tk + przysp * przysp * tk * tk
								* a_r / 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne przyspieszanie
					next_position = start_position + k * (node_counter * v_p
							* tk + node_counter * node_counter * a_r * tk * tk
							/ 2);
				}

			} else if (type == lib::RELATIVE) {//tryb relatywny
				if (przysp > (node_counter - 1) && przysp < (node_counter)) {//przyspieszanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku

						next_position = k * ((1 - node_counter + przysp) * v_r
								* tk - (1 - node_counter + przysp) * (1
								- node_counter + przysp) * a_r * tk * tk / 2
								+ jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk - a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp
									- jedn) * tk - tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn
									- przysp) * a_r / 2);
						} else {//przyspiesznie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk + a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp
									- jedn) * tk + tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn
									- przysp) * a_r / 2);
						}
					} else {//przyspieszenie + poczatek jednostajnego
						//printf("przyspieszanie wchodzi na jednostajny, macrostep liczony od 1: %d\n", node_counter);
						//next_position = start_position + k * ((node_counter-1)*v_p*tk + (node_counter-1)*(node_counter-1)*a_r*tk*tk/2);
						//next_position += k * (v_p*tk + a_r*tk*tk/2 + (v_p+a_r*(node_counter-1) *tk)) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp);
						next_position = k * (v_r * (1 - node_counter + przysp)
								* tk - (1 - node_counter + przysp) * (1
								- node_counter + przysp) * tk * tk * a_r / 2
								+ v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne przyspieszanie
					next_position = k * (v_p * tk + (node_counter - 1) * a_r
							* tk * tk + (a_r * tk * tk) / 2);
				}
			}

		} else { //hamowanie w pierwszym etapie
			//printf(" ham1 %d ", node_counter);
			if (type == lib::ABSOLUTE) {
				if ((przysp) > node_counter && (przysp) < (node_counter + 1)) {

					if (przysp + jedn < node_counter) {

						next_position = start_position + k * (przysp * v_p * tk
								- przysp * przysp * a_r * tk * tk / 2 + jedn
								* tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k * (v_r * (node_counter - przysp
									- jedn) * tk - tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn
									- przysp) * a_r / 2);
							//next_position += k * (v_p*tk - a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk - a_r*tk*tk/2) * (node_counter - przysp - jedn);
						} else {//przyspieszanie w trzecim etapie
							next_position += k * (v_r * (node_counter - przysp
									- jedn) * tk + tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn
									- przysp) * a_r / 2);
							//next_position += k * (v_p*tk - a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk + a_r*tk*tk/2) * (node_counter - przysp - jedn);
						}
					} else {//hamowanie + poczatek jednostajnego
						//next_position = start_position + k * ((node_counter-1)*v_p*tk + (node_counter-1)*(node_counter-1)*a_r*tk*tk/2);
						//next_position += k * (v_p*tk - a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp);
						next_position = start_position + k * (v_p
								* node_counter * tk - przysp * przysp * tk * tk
								* a_r / 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne hamowanie
					next_position = start_position + k * (node_counter * tk
							* v_p - node_counter * node_counter * tk * tk * a_r
							/ 2);
				}

			} else if (type == lib::RELATIVE) {
				if (przysp > (node_counter - 1) && przysp < (node_counter)) {//hamowanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku
						next_position = k * ((1 - node_counter + przysp) * v_r
								* tk + (1 - node_counter + przysp) * (1
								- node_counter + przysp) * a_r * tk * tk / 2
								+ jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk - a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp
									- jedn) * tk - tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn
									- przysp) * a_r / 2);
						} else {//przyspieszanie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk + a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp
									- jedn) * tk + tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn
									- przysp) * a_r / 2);
						}
					} else {//hamowanie + poczatek jednostajnego
						//printf("przyspieszanie wchodzi na jednostajny, macrostep liczony od 1: %d\n", node_counter);
						//next_position = start_position + k * ((node_counter-1)*v_p*tk + (node_counter-1)*(node_counter-1)*a_r*tk*tk/2);
						//next_position += k * (v_p*tk + a_r*tk*tk/2 + (v_p+a_r*(node_counter-1) *tk)) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp);
						next_position = k * (v_r * (1 - node_counter + przysp)
								* tk + (1 - node_counter + przysp) * (1
								- node_counter + przysp) * tk * tk * a_r / 2
								+ v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne hamowanie
					next_position = k * (v_p * tk - ((node_counter - 1) * a_r
							* tk * tk + (a_r * tk * tk) / 2));
				}
			}
		}
		//printf("%f\t", next_position);
	} else if (node_counter <= przysp + jedn + 1) { // drugi etap - ruch jednostajny
		//printf(" jedn %d ", node_counter);
		if (type == lib::ABSOLUTE) {
			if ((przysp + jedn) > (node_counter - 1) && (przysp + jedn)
					< (node_counter)) {//jednostajny wchodzi w faze trzecia

				next_position = start_position + k * (s_przysp + s_jedn);

				if (v_r > v_k) {//hamowanie w 3 etapie
					//printf("wchodzi macrostep: %d\n", node_counter);
					next_position += k * (v_r * (node_counter - jedn - przysp)
							* tk - (node_counter - jedn - przysp)
							* (node_counter - jedn - przysp) * tk * tk * a_r
							/ 2);
				} else {//przyspieszanie w 3 etapie
					next_position += k * (v_r * (node_counter - jedn - przysp)
							* tk + (node_counter - jedn - przysp)
							* (node_counter - jedn - przysp) * tk * tk * a_r
							/ 2);
				}
			} else { //normalny jednostajny
				next_position = start_position + k * (s_przysp + ((node_counter
						- przysp) * tk) * v_r);
			}
			//printf("next_pos: %f\n", next_position);
		} else if (type == lib::RELATIVE) {
			if ((przysp + jedn) > (node_counter - 1) && (przysp + jedn)
					< (node_counter)) {//jednostajny wchodzi w faze trzecia

				next_position = v_r * tk * (1 - node_counter + przysp + jedn);

				if (v_r > v_k) {//hamowanie w 3 etapie
					//printf("wchodzi macrostep: %d\n", node_counter);
					next_position += k * (v_r * (node_counter - jedn - przysp)
							* tk - (node_counter - jedn - przysp)
							* (node_counter - jedn - przysp) * tk * tk * a_r
							/ 2);
				} else {//przyspieszanie w 3 etapie
					next_position += k * (v_r * (node_counter - jedn - przysp)
							* tk + (node_counter - jedn - przysp)
							* (node_counter - jedn - przysp) * tk * tk * a_r
							/ 2);
				}
			} else { //normalny jednostajny
				next_position = k * v_r * tk;
			}
			//printf("%f\t", next_position);
		}

	} else if (node_counter <= interpolation_node_no) { //trzeci etap

		if (v_k <= v_r) { //hamowanie w trzecim etapie
			//printf(" ham2 %d ", node_counter);
			if (type == lib::ABSOLUTE) {
				//printf("start: %f\t node_counter: %d\t s przysp: %f\t s_jedn: %f\n", start_position, node_counter, s_przysp, s_jedn);
				//printf("next_pos: %f\n", next_position);
				next_position = start_position + k * (s_przysp + s_jedn
						+ ((node_counter - jedn - przysp) * tk) * v_r
						- ((node_counter - jedn - przysp) * tk)
								* ((node_counter - jedn - przysp) * tk) * a_r
								/ 2);
				//printf("next_pos hamowanie: %f\n", next_position);
			} else if (type == lib::RELATIVE) {
				next_position = k * ((v_k + (interpolation_node_no
						- node_counter) * a_r * tk) * tk + (a_r * tk * tk) / 2);
			}
			//printf("next pos: %f\t node: %d\t", next_position, node_counter);
		} else { //przyspieszanie w trzecim etapie
			//printf(" przysp2 %d ", node_counter);
			if (type == lib::ABSOLUTE) {
				next_position = start_position + k * (s_przysp + s_jedn
						+ ((node_counter - jedn - przysp) * tk) * v_r + a_r
						* ((node_counter - jedn - przysp) * tk)
						* ((node_counter - jedn - przysp) * tk) / 2);
			} else if (type == lib::RELATIVE) {
				next_position = k * ((v_k - (interpolation_node_no
						- node_counter) * a_r * tk) * tk - (a_r * tk * tk) / 2);
			}
		}
		//printf("%f\t", next_position);
	}

	//printf("next pos: %f\t node: %d\t", next_position, node_counter);
	//flushall();
	return next_position;
}

void smooth::generate_coords() {

	double coordinate[MAX_SERVOS_NR];
	int private_node_counter = 1;
	initiate_pose_list();
	flush_coordinate_list();
	for (int j = 0; j < pose_list_length(); j++) {

		//printf("start pos w generate_cords: %f\t w osi: %d\n",pose_list_iterator->start_position[0], 0);

		for (int z = 0; z < pose_list_iterator->interpolation_node_no; z++) {
			for (int i = 0; i < MAX_SERVOS_NR; i++) {
				if (type == lib::ABSOLUTE && fabs(
						pose_list_iterator->start_position[i]
								- pose_list_iterator->coordinates[i])
						< distance_eps) {
					coordinate[i] = pose_list_iterator->start_position[i]; //dla drogi 0
				} else if (type == lib::RELATIVE && eq(
						pose_list_iterator->coordinates[i], 0)) {//dla drogi 0 w relative
					coordinate[i] = 0;
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
			coordinate_list.push_back(coordinates(coordinate));
		}
		private_node_counter = 1;
		next_pose_list_ptr();
	}

	trajectory_generated = true;
	//printowanie listy coordinate
	initiate_coordinate_list();
	for (int m = 0; m < coordinate_list.size(); m++) {
		printf("makrokrok: %d\t", m);
		for (int n = 0; n < 8; n++) {
			printf("%f\t", coordinate_list_iterator->coordinate[n]);
		}
		printf("\n");
		flushall();
		coordinate_list_iterator++;
	}
	//printf("\ngenerate_cords\n");
}

void smooth::send_coordinates() {
	//lib::Xyz_Euler_Zyz_vector tmp_euler_vector;
	double gripper_position;
	double tk = 10 * STEP;
	int i; //licznik petli
	int gripp; //os grippera

	the_robot->ecp_command.instruction.instruction_type = lib::SET; //ustawienie parametrow ruchu w edp_data
	the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION; //ponizej w caseach jest dalsze ustawianie
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	switch (td.arm_type) {

	case lib::ECP_XYZ_EULER_ZYZ:

		homog_matrix.set_from_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector(
				coordinate_list_iterator->coordinate));
		homog_matrix.get_frame_tab(
				the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

		gripp = 6;

		break;

	case lib::ECP_XYZ_ANGLE_AXIS:

		homog_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(
				coordinate_list_iterator->coordinate));
		homog_matrix.get_frame_tab(
				the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

		gripp = 6;

		break;

	case lib::ECP_JOINT:

		for (i = 0; i < MAX_SERVOS_NR; i++) {
			the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
					= coordinate_list_iterator->coordinate[i];
		}

		if (the_robot->robot_name == lib::ROBOT_IRP6OT_M) {
			gripp = 7;
		} else if (the_robot->robot_name == lib::ROBOT_IRP6P_M) {
			gripp = 6;
		}

		break;

	case lib::ECP_MOTOR:

		for (i = 0; i < MAX_SERVOS_NR; i++) {
			the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
					= coordinate_list_iterator->coordinate[i];
		}

		if (the_robot->robot_name == lib::ROBOT_IRP6OT_M) {
			gripp = 7;
		} else if (the_robot->robot_name == lib::ROBOT_IRP6P_M) {
			gripp = 6;
		}

		break;

	default:
		throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch

	//gripper
	if (type == lib::RELATIVE) {
		gripper_position = pose_list_iterator->k[gripp] * ((tk)
				* pose_list_iterator->v_grip);

		printf("gripper_position: %f\n ", gripper_position);

		//printf(" %f \t", gripper_position);
		if ((node_counter * gripper_position
				> pose_list_iterator->coordinates[gripp]
				&& pose_list_iterator->k[gripp] == -1) || (node_counter
				* gripper_position < pose_list_iterator->coordinates[gripp]
				&& pose_list_iterator->k[gripp] == 1)) {
			//printf("git");
			/*
			 the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
			 = gripper_position;
			 */
		} else {
			/*
			 the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
			 = 0;
			 */
		}
	} else {

		gripper_position = pose_list_iterator->start_position[gripp]
				+ pose_list_iterator->k[gripp] * ((node_counter * tk)
						* pose_list_iterator->v_grip);
		//printf(" %f ", gripper_position);
		if ((gripper_position > pose_list_iterator->coordinates[gripp]
				&& pose_list_iterator->k[gripp] == -1) || (gripper_position
				< pose_list_iterator->coordinates[gripp]
				&& pose_list_iterator->k[gripp] == 1)) {
			//printf("git");
			/*
			 the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
			 = gripper_position;
			 */
		} else {
			printf("no gripp\t");
			/*
			 the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
			 = pose_list_iterator->coordinates[gripp];
			 */
		}
	}

	for (int n = 0; n < gripp; n++) {
		printf("%f\t", coordinate_list_iterator->coordinate[n]);
	}

	printf("%f\n", gripper_position);

	flushall();

	coordinate_list_iterator++;
}

//set necessary instructions, and other data for preparing the robot
bool smooth::first_step() { //wywolywane tylko raz w calej trajektorii

	//flush_coordinate_list();
	int gripp; //licznik petli
	initiate_pose_list();
	td.arm_type = pose_list_iterator->arm_type;

	first_interval = true;//to chyba nie jest potrzebne bo ustawianie first_interval jest takze na poczatku calculate()

	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.instruction_type = lib::GET;

	switch (td.arm_type) {

	case lib::ECP_MOTOR:
		the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
		the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
		if (type == lib::RELATIVE) {
			the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
		} else {
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		}
		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		break;
	case lib::ECP_JOINT:
		the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
		the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
		if (type == lib::RELATIVE) {
			the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
		} else {
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		}
		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		/*the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
		 for (int i=0; i<6; i++)
		 {
		 the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		 }*/
		break;

	case lib::ECP_XYZ_EULER_ZYZ:
		the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
		the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
		if (type == lib::RELATIVE) {
			the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
			the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
			for (int i = 0; i < 6; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.behaviour[i]
						= lib::UNGUARDED_MOTION;
			}
		} else {
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		}
		break;
	case lib::ECP_XYZ_ANGLE_AXIS:
		the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
		the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
		if (type == lib::RELATIVE) {
			the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
			the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
			for (int i = 0; i < 6; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.behaviour[i]
						= lib::UNGUARDED_MOTION;
			}
		} else {
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		}
		break;

	default:
		throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end : switch ( td.arm_type )

	return true;
} // end: bool ecp_smooth_generator::first_step ( )

bool smooth::next_step() {

	if (!trajectory_generated && !trajectory_calculated) {
		calculate(); //wypelnienie pozostalych zmiennych w liscie pose_list
		generate_coords(); //wypelnienie listy coordinate_list (lista kolejnych pozycji w makrokrokach)
		initiate_pose_list(); //ustawienie iteratora pose_list na poczatek
		initiate_coordinate_list(); //ustawienie iteratora coordinate_list na poczatek
	}

	if (!trajectory_generated || !trajectory_calculated) {
		return false;
	}

	if (node_counter == pose_list_iterator->interpolation_node_no) {//czy poprzedni makrokrok byl ostatnim

		if (is_last_list_element()) { //ostatni punkt (koniec listy pozycji pose_list)
			//if (coordinate_list_itarator)
			send_coordinates();
			reset();
			return false;

		} else {//lista pozycji pose_list nie jest skonczona wiec idziemy do nastepnego punktu

			send_coordinates();
			node_counter = 0; //TODO przestestowac, sprawdzic czy 1 czy 0
			next_pose_list_ptr();
			td.interpolation_node_no
					= pose_list_iterator->interpolation_node_no;
		}
	} else {

		send_coordinates();

	}// end: if
	return true;

} // end: bool ecp_smooth_generator::next_step ( )

void smooth::calculate(void) { //zeby wrocic do starego trybu relative nalezy stad usunac wszystkie warunki na type i zostawic opcje dla type = 1

	double s[MAX_SERVOS_NR];//droga w danych stopniach swobody w jednym ruchu
	double s_temp1[MAX_SERVOS_NR], s_temp2[MAX_SERVOS_NR];
	double t_temp1, t_temp2;
	double t[MAX_SERVOS_NR];
	lib::Xyz_Euler_Zyz_vector tmp_euler_vector;
	lib::Xyz_Angle_Axis_vector tmp_angle_axis_vector;

	double t_max; //nadluzszy czas ruchu w jednej osi w jednym ruchu
	int i;
	double tk = 10 * STEP; //czas jednego makrokroku
	int gripp; //os grippera

	trajectory_calculated = false;
	//TODO dorobic zabezpieczenia dla 0 predkosci w osmej wspolrzednej postumenta i w angle axes
	printf("CALCULATE\n");
	double v_r_next[MAX_SERVOS_NR];//predkosc kolejnego ruchu

	double temp;//generalny temp do wszystkiego

	initiate_pose_list();//ustawianie wskaznika listy pozycji na poczatek

	first_interval = true;

	td.internode_step_no = 10;
	td.value_in_step_no = td.internode_step_no - 2;

	for (int j = 0; j < pose_list_length(); j++) {

		td.arm_type = pose_list_iterator->arm_type;

		if (first_interval) {//pierwsza pozycja ruchu
			//wywoluje sie tylko raz, przy pierwzszym ruchu w trajektorii, musi byc rozroznione, gdyz tutaj v_p jest = 0 a pozycja jest
			// ...odczytywana z ramienia...

			pose_list_iterator->pos_num = j;

			switch (td.arm_type) {

			case lib::ECP_XYZ_EULER_ZYZ:
				gripp = 6;
				homog_matrix.set_from_frame_tab(
						the_robot->reply_package.arm.pf_def.arm_frame);

				//homog_matrix.get_xyz_euler_zyz(pose_list_iterator->start_position);


				homog_matrix.get_xyz_euler_zyz(tmp_euler_vector);
				tmp_euler_vector.to_table(pose_list_iterator->start_position);

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];

					if (!is_last_list_element()) { //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr(); //pozycje startowa nowego ruchu
					}
				}
				temp = pose_list_iterator->coordinates[gripp];
				/*	pose_list_iterator->start_position[gripp]
				 = the_robot->reply_package.arm.pf_def.gripper_coordinate;*/
				if (!is_last_list_element()) { //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_list_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr(); //pozycje startowa nowego ruchu
				}
				pose_list_iterator->start_position[gripp + 1] = 0.0;

				for (i = 0; i < MAX_SERVOS_NR; i++) {

					if (v_max_zyz[i] == 0 || a_max_zyz[i] == 0
							|| pose_list_iterator->v[i] == 0
							|| pose_list_iterator->a[i] == 0) {
						sr_ecp_msg.message(
								"One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR,
								INVALID_MP_COMMAND);
					}

					pose_list_iterator->v_r[i] = v_max_zyz[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_zyz[i]
							* pose_list_iterator->a[i];

					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0
					if (type == lib::ABSOLUTE) {
						if (pose_list_iterator->coordinates[i]
								- pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
							pose_list_iterator->k[i] = -1; //zapisanie kierunku nastepnych ruchow dokonywane jest dalej
						} else {
							pose_list_iterator->k[i] = 1;
						}
					} else if (type == lib::RELATIVE) {
						if (pose_list_iterator->coordinates[i] < 0) {
							pose_list_iterator->k[i] = -1;
						} else {
							pose_list_iterator->k[i] = 1;
						}
					}
				}

				break;

			case lib::ECP_XYZ_ANGLE_AXIS:
				gripp = 6;

				homog_matrix.set_from_frame_tab(
						the_robot->reply_package.arm.pf_def.arm_frame);

				homog_matrix.get_xyz_angle_axis(tmp_angle_axis_vector);
				tmp_angle_axis_vector.to_table(
						pose_list_iterator->start_position);

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) { //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr(); //pozycje startowa nowego ruchu
					}
				}
				temp = pose_list_iterator->coordinates[gripp];
				/*
				 pose_list_iterator->start_position[gripp]
				 = the_robot->reply_package.arm.pf_def.gripper_coordinate;
				 */
				if (!is_last_list_element()) { //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_list_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr(); //pozycje startowa nowego ruchu
				}
				pose_list_iterator->start_position[gripp + 1] = 0.0;

				for (i = 0; i < MAX_SERVOS_NR; i++) {

					if (v_max_aa[i] == 0 || a_max_aa[i] == 0
							|| pose_list_iterator->v[i] == 0
							|| pose_list_iterator->a[i] == 0) {
						sr_ecp_msg.message(
								"One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR,
								INVALID_MP_COMMAND);
					}

					pose_list_iterator->v_r[i] = v_max_aa[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_aa[i]
							* pose_list_iterator->a[i];

					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if (type == lib::ABSOLUTE) {
						if (pose_list_iterator->coordinates[i]
								- pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
							pose_list_iterator->k[i] = -1; //zapisanie kierunku nastepnych ruchow dokonywane jest dalej
						} else {
							pose_list_iterator->k[i] = 1;
						}
					} else if (type == lib::RELATIVE) {
						if (pose_list_iterator->coordinates[i] < 0) {
							pose_list_iterator->k[i] = -1;
						} else {
							pose_list_iterator->k[i] = 1;
						}
					}
				}

				break;

			case lib::ECP_JOINT:
				if (the_robot->robot_name == lib::ROBOT_IRP6OT_M) {
					gripp = 7;
				} else if (the_robot->robot_name == lib::ROBOT_IRP6P_M) {
					gripp = 6;
				}

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					pose_list_iterator->start_position[i]
							= the_robot->reply_package.arm.pf_def.arm_coordinates[i];//pierwsze przypisanie start_position
					if (!is_last_list_element()) { //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr(); //pozycje startowa nowego ruchu
					}
				}
				temp = pose_list_iterator->coordinates[gripp];
				pose_list_iterator->start_position[gripp]
						= the_robot->reply_package.arm.pf_def.arm_coordinates[gripp];
				if (!is_last_list_element()) { //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_list_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr(); //pozycje startowa nowego ruchu
				}
				if (gripp < (MAX_SERVOS_NR - 1)) {
					pose_list_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (!(the_robot->robot_name == lib::ROBOT_IRP6P_M && i
							== (MAX_SERVOS_NR - 1))) {
						if (v_max_joint[i] == 0 || a_max_joint[i] == 0
								|| pose_list_iterator->v[i] == 0
								|| pose_list_iterator->a[i] == 0) {
							sr_ecp_msg.message(
									"One or more of 'v' or 'a' values is 0");
							throw ECP_error(lib::NON_FATAL_ERROR,
									INVALID_MP_COMMAND);
						}
					}

					pose_list_iterator->v_r[i] = v_max_joint[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_joint[i]
							* pose_list_iterator->a[i];

					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if (type == lib::ABSOLUTE) {
						if (pose_list_iterator->coordinates[i]
								- pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
							pose_list_iterator->k[i] = -1; //zapisanie kierunku nastepnych ruchow dokonywane jest dalej
						} else {
							pose_list_iterator->k[i] = 1;
						}
					} else if (type == lib::RELATIVE) {
						if (pose_list_iterator->coordinates[i] < 0) {
							pose_list_iterator->k[i] = -1;
						} else {
							pose_list_iterator->k[i] = 1;
						}
					}
				}

				break;

			case lib::ECP_MOTOR:
				if (the_robot->robot_name == lib::ROBOT_IRP6OT_M) {
					gripp = 7;
				} else if (the_robot->robot_name == lib::ROBOT_IRP6P_M) {
					gripp = 6;
				}

				for (i = 0; i < gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					pose_list_iterator->start_position[i]
							= the_robot->reply_package.arm.pf_def.arm_coordinates[i];//pierwsze przypisanie start_position
					if (!is_last_list_element()) { //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr(); //pozycje startowa nowego ruchu
					}
				}
				temp = pose_list_iterator->coordinates[gripp];
				pose_list_iterator->start_position[gripp]
						= the_robot->reply_package.arm.pf_def.arm_coordinates[gripp];
				if (!is_last_list_element()) { //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_list_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr(); //pozycje startowa nowego ruchu
				}
				if (gripp < (MAX_SERVOS_NR - 1)) {
					pose_list_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_motor[i] == 0 || a_max_motor[i] == 0
							|| pose_list_iterator->v[i] == 0
							|| pose_list_iterator->a[i] == 0) {
						sr_ecp_msg.message(
								"One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR,
								INVALID_MP_COMMAND);
					}

					pose_list_iterator->v_r[i] = v_max_motor[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_motor[i]
							* pose_list_iterator->a[i];

					pose_list_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if (type == lib::ABSOLUTE) {
						if (pose_list_iterator->coordinates[i]
								- pose_list_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
							pose_list_iterator->k[i] = -1; //zapisanie kierunku nastepnych ruchow dokonywane jest dalej
						} else {
							pose_list_iterator->k[i] = 1;
						}
					} else if (type == lib::RELATIVE) {
						if (pose_list_iterator->coordinates[i] < 0) {
							pose_list_iterator->k[i] = -1;
						} else {
							pose_list_iterator->k[i] = 1;
						}
					}
				}

				break;

			default:
				throw ECP_error(lib::NON_FATAL_ERROR,
						INVALID_POSE_SPECIFICATION);

			} // end: switch

			first_interval = false;

		} else { // end: if(first_interval)

			switch (td.arm_type) {
			//tutaj jestesmy jeszcze ciagle w poprzedniej pozycji pose_list

			case lib::ECP_XYZ_EULER_ZYZ:
				gripp = 6;
				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr(); //predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

				pose_list_iterator->pos_num = j;
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr(); //pozycje startowa nowego ruchu
					}
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_zyz[i] == 0 || a_max_zyz[i] == 0
							|| pose_list_iterator->v[i] == 0
							|| pose_list_iterator->a[i] == 0) {
						sr_ecp_msg.message(
								"One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR,
								INVALID_MP_COMMAND);
					}
					pose_list_iterator->v_r[i] = v_max_zyz[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_zyz[i]
							* pose_list_iterator->a[i];
				}

				break;

			case lib::ECP_XYZ_ANGLE_AXIS:
				gripp = 6;
				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr(); //predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

				pose_list_iterator->pos_num = j;
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr(); //pozycje startowa nowego ruchu
					}
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_aa[i] == 0 || a_max_aa[i] == 0
							|| pose_list_iterator->v[i] == 0
							|| pose_list_iterator->a[i] == 0) {
						sr_ecp_msg.message(
								"One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR,
								INVALID_MP_COMMAND);
					}
					pose_list_iterator->v_r[i] = v_max_aa[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_aa[i]
							* pose_list_iterator->a[i];
				}

				break;

			case lib::ECP_JOINT:

				if (the_robot->robot_name == lib::ROBOT_IRP6OT_M) {
					gripp = 7;
				} else if (the_robot->robot_name == lib::ROBOT_IRP6P_M) {
					gripp = 6;
				}

				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr(); //predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

				pose_list_iterator->pos_num = j;
				for (i = 0; i <= gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr(); //pozycje startowa nowego ruchu
					}
				}

				if (gripp < (MAX_SERVOS_NR - 1)) {//jesli postument
					pose_list_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_joint[i] == 0 || a_max_joint[i] == 0
							|| pose_list_iterator->v[i] == 0
							|| pose_list_iterator->a[i] == 0) {
						sr_ecp_msg.message(
								"One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR,
								INVALID_MP_COMMAND);
					}
					pose_list_iterator->v_r[i] = v_max_joint[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_joint[i]
							* pose_list_iterator->a[i];
				}

				break;

			case lib::ECP_MOTOR:

				if (the_robot->robot_name == lib::ROBOT_IRP6OT_M) {
					gripp = 7;
				} else if (the_robot->robot_name == lib::ROBOT_IRP6P_M) {
					gripp = 6;
				}

				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr(); //predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

				pose_list_iterator->pos_num = j;
				for (i = 0; i <= gripp; i++) {
					temp = pose_list_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_list_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr(); //pozycje startowa nowego ruchu
					}
				}

				if (gripp < (MAX_SERVOS_NR - 1)) {//jesli postument
					pose_list_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_motor[i] == 0 || a_max_motor[i] == 0
							|| pose_list_iterator->v[i] == 0
							|| pose_list_iterator->a[i] == 0) {
						sr_ecp_msg.message(
								"One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR,
								INVALID_MP_COMMAND);
					}
					pose_list_iterator->v_r[i] = v_max_motor[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_motor[i]
							* pose_list_iterator->a[i];
				}

				break;

			default:
				throw ECP_error(lib::NON_FATAL_ERROR,
						INVALID_POSE_SPECIFICATION);
			}
		} //end else (first interval)

		for (i = 0; i < MAX_SERVOS_NR; i++) {//zapisanie coordinate_delta
			if (type == lib::ABSOLUTE) {
				td.coordinate_delta[i] = pose_list_iterator->coordinates[i]
						- pose_list_iterator->start_position[i];
			} else if (type == lib::RELATIVE) {
				td.coordinate_delta[i] = pose_list_iterator->coordinates[i];
			}
		}

		for (i = 0; i < MAX_SERVOS_NR; i++) {//zapisanie v_r_next dla aktualnego ruchu i k dla nastepnego ruchu
			if (is_last_list_element()) {
				v_r_next[i] = 0.0;
			} else {
				temp = pose_list_iterator->k[i];
				next_pose_list_ptr();

				if (type == lib::ABSOLUTE) {
					if (eq(pose_list_iterator->coordinates[i],
							pose_list_iterator->start_position[i])) {
						pose_list_iterator->k[i] = -temp;//jesli droga jest rowna 0 w nastepnym ruchu to kierunek ustawiany jest na przeciwny aby robot zwolnil do 0
					} else if (pose_list_iterator->coordinates[i]
							- pose_list_iterator->start_position[i] < 0) {//nadpisanie k dla nastepnego ruchu
						pose_list_iterator->k[i] = -1;
					} else {
						pose_list_iterator->k[i] = 1;
					}
				} else if (type == lib::RELATIVE) {
					if (eq(pose_list_iterator->coordinates[i], 0)) {
						pose_list_iterator->k[i] = -temp;//jesli droga jest rowna 0 w nastepnym ruchu to kierunek ustawiany jest na przeciwny aby robot zwolnil do 0
					} else if (pose_list_iterator->coordinates[i] < 0) {//nadpisanie k dla nastepnego ruchu
						pose_list_iterator->k[i] = -1;
					} else {
						pose_list_iterator->k[i] = 1;
					}
				}

				switch (td.arm_type) {

				case lib::ECP_XYZ_EULER_ZYZ:
					pose_list_iterator->v_r[i] = v_max_zyz[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_zyz[i]
							* pose_list_iterator->a[i];

					break;

				case lib::ECP_XYZ_ANGLE_AXIS:
					pose_list_iterator->v_r[i] = v_max_aa[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_aa[i]
							* pose_list_iterator->a[i];

					break;

				case lib::ECP_MOTOR:
					pose_list_iterator->v_r[i] = v_max_motor[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_motor[i]
							* pose_list_iterator->a[i];

					break;

				case lib::ECP_JOINT:
					pose_list_iterator->v_r[i] = v_max_joint[i]
							* pose_list_iterator->v[i];
					pose_list_iterator->a_r[i] = a_max_joint[i]
							* pose_list_iterator->a[i];

					break;
				default:
					throw ECP_error(lib::NON_FATAL_ERROR,
							INVALID_POSE_SPECIFICATION);
				}

				if (pose_list_iterator->k[i] != temp) {
					v_r_next[i] = 0;
				} else {
					v_r_next[i] = pose_list_iterator->v_r[i];
				}

				prev_pose_list_ptr();
			}
		}
		// ==================================== poczatek oliczen ========================================

		t_max = 0;

		//obliczenie drogi dla kazdej osi
		for (i = 0; i < MAX_SERVOS_NR; i++) {
			if (type == lib::ABSOLUTE) {
				s[i] = fabs(pose_list_iterator->coordinates[i]
						- pose_list_iterator->start_position[i]);
			} else if (type == lib::RELATIVE) {
				s[i] = fabs(pose_list_iterator->coordinates[i]);//tryb relative
			}
		}
		//sprawdzanie czy droga w etapach przyspieszenia i hamowania nie jest wieksza niz droga calego ruchu

		//warunki na modele ruchu dla wszystkich osi
		for (i = 0; i < MAX_SERVOS_NR; i++) { //petla w ktorej obliczany jest czas dla kazdej osi i sprawdzane jest czy da sie wykonac ruch w zalozonych etapach
			printf("=============================================\n");
			printf("v_p: %f\t v_r: %f\t v_r_next: %f\t a_r: %f\t v_k: %f\n",
					pose_list_iterator->v_p[i], pose_list_iterator->v_r[i],
					v_r_next[i], pose_list_iterator->a_r[i],
					pose_list_iterator->v_k[i]);
			flushall();

			if (s[i] < distance_eps) {//najmniejsza wykrywalna droga
				//printf("droga 0 %d\n", i);
				pose_list_iterator->model[i] = 0; //kinematic_model_with_tool 0... brak ruchu
				t[i] = 0;
				pose_list_iterator->s_przysp[i] = 0;
				pose_list_iterator->s_jedn[i] = 0;
				pose_list_iterator->v_k[i] = 0;
				continue;
			}

			if (pose_list_iterator->v_p[i] < pose_list_iterator->v_r[i]
					&& v_r_next[i] < pose_list_iterator->v_r[i]) { //pierwszy kinematic_model_with_tool
				//printf("pierwszy kinematic_model_with_tool w osi %d\n", i);

				pose_list_iterator->model[i] = 1;

				s_temp1[i] = pose_list_iterator->v_p[i]
						* (pose_list_iterator->v_r[i]
								- pose_list_iterator->v_p[i])
						/ pose_list_iterator->a_r[i] + (0.5
						* pose_list_iterator->a_r[i]
						* ((pose_list_iterator->v_r[i]
								- pose_list_iterator->v_p[i])
								/ pose_list_iterator->a_r[i])
						* ((pose_list_iterator->v_r[i]
								- pose_list_iterator->v_p[i])
								/ pose_list_iterator->a_r[i]));
				s_temp2[i] = pose_list_iterator->v_r[i]
						* (pose_list_iterator->v_r[i] - v_r_next[i])
						/ pose_list_iterator->a_r[i] - (0.5
						* pose_list_iterator->a_r[i]
						* ((pose_list_iterator->v_r[i] - v_r_next[i])
								/ pose_list_iterator->a_r[i])
						* ((pose_list_iterator->v_r[i] - v_r_next[i])
								/ pose_list_iterator->a_r[i]));

				t_temp1 = (pose_list_iterator->v_r[i]
						- pose_list_iterator->v_p[i])
						/ pose_list_iterator->a_r[i];
				t_temp2 = (pose_list_iterator->v_r[i] - v_r_next[i])
						/ pose_list_iterator->a_r[i];

				pose_list_iterator->v_k[i] = v_r_next[i];

				//printf("s_temp1: %f\ts_temp2: %f\n", s_temp1[i], s_temp2[i]);
				//printf("t_temp1: %f\tt_temp2: %f\n", t_temp1, t_temp2);

				if (s_temp1[i] + s_temp2[i] > s[i]) {
					printf("redukcja predkosci w osi %d\n", i);
					//TODO tutaj wstawic optymalizacje czasu
					optimize_time1(pose_list_iterator, i, s[i]);//nastepuje zapisanie czasu, mozliwe wywolanie vp_reduction lub vk_reduction wewnatrz
					if (trajectory_calculated == true) {
						return;
					}
					t[i] = pose_list_iterator->t;
					//printf("t[i]: %f\n", t[i]);
					//t[i] = t_temp1 + t_temp2;
					pose_list_iterator->t = t[i];
					reduction_model_1(pose_list_iterator, i, s[i]);
					//printf("1 v_r: %f\n", pose_list_iterator->v_r[i]);
					if (trajectory_calculated == true) {
						return;
					}

				} else {//droga przyspieszenia i opoznienia nie przekracza drogi ruchu

					t[i] = t_temp1 + t_temp2 + (s[i]
							- (s_temp1[i] + s_temp2[i]))
							/ pose_list_iterator->v_r[i];
					pose_list_iterator->s_przysp[i] = s_temp1[i];
					pose_list_iterator->s_jedn[i] = s[i] - s_temp2[i]
							- s_temp1[i];
				}

			} else if (pose_list_iterator->v_p[i] < pose_list_iterator->v_r[i]
					&& (v_r_next[i] > pose_list_iterator->v_r[i] || eq(
							v_r_next[i], pose_list_iterator->v_r[i]))) { // drugi kinematic_model_with_tool
				//printf("drugi kinematic_model_with_tool w osi %d\n", i);

				pose_list_iterator->model[i] = 2;
				s_temp1[i] = pose_list_iterator->v_p[i]
						* (pose_list_iterator->v_r[i]
								- pose_list_iterator->v_p[i])
						/ pose_list_iterator->a_r[i] + (0.5
						* pose_list_iterator->a_r[i]
						* ((pose_list_iterator->v_r[i]
								- pose_list_iterator->v_p[i])
								/ pose_list_iterator->a_r[i])
						* ((pose_list_iterator->v_r[i]
								- pose_list_iterator->v_p[i])
								/ pose_list_iterator->a_r[i]));
				t_temp1 = (pose_list_iterator->v_r[i]
						- pose_list_iterator->v_p[i])
						/ pose_list_iterator->a_r[i];

				pose_list_iterator->v_k[i] = pose_list_iterator->v_r[i];

				//printf("s_temp1: %f\n", s_temp1[i]);
				//printf("t_temp1: %f\n", t_temp1);

				if (s_temp1[i] > s[i]) {
					optimize_time2(pose_list_iterator, i, s[i]);
					t[i] = pose_list_iterator->t;
					//t[i] = t_temp1;
					vk_reduction(pose_list_iterator, i, s[i], t[i]);

					if (trajectory_calculated == true) {
						return;
					}

				} else {

					t[i] = t_temp1 + (s[i] - s_temp1[i])
							/ pose_list_iterator->v_r[i];
					pose_list_iterator->s_przysp[i] = s_temp1[i];
					pose_list_iterator->s_jedn[i] = s[i] - s_temp1[i];
				}

			} else if (eq(pose_list_iterator->v_p[i],
					pose_list_iterator->v_r[i]) && (v_r_next[i]
					> pose_list_iterator->v_r[i] || eq(v_r_next[i],
					pose_list_iterator->v_r[i]))) { //trzeci kinematic_model_with_tool
				//printf("trzeci kinematic_model_with_tool w osi %d\n", i);

				pose_list_iterator->model[i] = 3;

				pose_list_iterator->s_przysp[i] = 0;

				if (v_r_next[i] > pose_list_iterator->v_r[i] || (eq(
						v_r_next[i], 0) && eq(pose_list_iterator->v_r[i], 0))) { //ten przypadek wystepuje w rekurencji, po redukcji (prawdopodobnie po ostatnich zmianach juz nie wystepuje...)

					t[i] = pose_list_iterator->t;
					if (v_r_next[i] != 0) {
						pose_list_iterator->v_k[i] = v_r_next[i];
					}

					if (eq(pose_list_iterator->v_r[i], 0)) {
						pose_list_iterator->s_jedn[i] = 0;
					} else {
						t_temp1 = (pose_list_iterator->v_k[i]
								- pose_list_iterator->v_r[i])
								/ pose_list_iterator->a_r[i];
						pose_list_iterator->s_jedn[i]
								= s[i]
										- ((pose_list_iterator->a_r[i]
												* t_temp1 * t_temp1) / 2
												+ (pose_list_iterator->v_r[i]
														* t_temp1));
					}

				} else {
					t[i] = s[i] / pose_list_iterator->v_r[i];

					//printf("t: %f\n", t[i]);
					pose_list_iterator->s_jedn[i] = s[i];
					pose_list_iterator->v_k[i] = pose_list_iterator->v_r[i];
				}
			} else if (eq(pose_list_iterator->v_p[i],
					pose_list_iterator->v_r[i]) && v_r_next[i]
					< pose_list_iterator->v_r[i]) { //czwarty kinematic_model_with_tool
				//printf("czwarty kinematic_model_with_tool w osi %d\n", i);

				pose_list_iterator->model[i] = 4;

				s_temp1[i] = pose_list_iterator->v_r[i]
						* (pose_list_iterator->v_r[i] - v_r_next[i])
						/ pose_list_iterator->a_r[i] - (0.5
						* pose_list_iterator->a_r[i]
						* ((pose_list_iterator->v_r[i] - v_r_next[i])
								/ pose_list_iterator->a_r[i])
						* ((pose_list_iterator->v_r[i] - v_r_next[i])
								/ pose_list_iterator->a_r[i]));
				t_temp1 = (pose_list_iterator->v_r[i] - v_r_next[i])
						/ pose_list_iterator->a_r[i];

				pose_list_iterator->v_k[i] = v_r_next[i];

				//printf("s_temp1: %f\n", s_temp1[i]);
				//printf("t_temp1: %f\n", t_temp1);

				if (s_temp1[i] > s[i]) {
					optimize_time4(pose_list_iterator, i, s[i]);
					t[i] = pose_list_iterator->t;
					//t[i] = t_temp1;
					/*vp_reduction(pose_list_iterator, i, s[i], t[i]);
					 if (trajectory_calculated == true) {
					 return;
					 }*/
					pose_list_iterator->s_przysp[i] = 0;
					pose_list_iterator->s_jedn[i] = 0;

				} else {

					t[i] = t_temp1 + (s[i] - s_temp1[i])
							/ pose_list_iterator->v_r[i];
					pose_list_iterator->s_przysp[i] = 0;
					pose_list_iterator->s_jedn[i] = s[i] - s_temp1[i];
				}

			} else {
				//printf("\n ten przypadek nigdy nie moze wystapic\n");
				//printf(" ********************** Error w osi %d *************************\n", i);
				//flushall();pose_list_iterator->t = t_max;
				sr_ecp_msg.message(
						"Unexpected calculation error 1. Save your trajectory and report bug");
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
			}

			//printf("t: %f\t t max: %f\n", t[i], t_max);

			if (t[i] > t_max) {//nadpisanie najdluzszego czasu
				t_max = t[i];
			}
			//printf("t_i: %f\n", t[i]);
		}
		//printf("t_max: %f\n", t_max);
		//kwantyzacja czasu
		if (ceil(t_max / tk) * tk != t_max)//zaokraglenie czasu do wielokrotnosci trwania makrokroku
		{
			t_max = ceil(t_max / tk);
			t_max = t_max * tk;
			//printf("t_max = %f\n", t_max);
			pose_list_iterator->t = t_max;
		}

		pose_list_iterator->interpolation_node_no = lround(t_max / tk);
		//printf("liczba makrokrokow w ruchu %d\n", pose_list_iterator->interpolation_node_no);

		for (i = 0; i < MAX_SERVOS_NR; i++) {//obliczanie przysp i jedn a takze ewentualna redukcja predkosci z powodu zbyt krotkiego czasu

			//printf("czas ruchu w osi %d = %f\n", i, t[i]);

			//if (s[i] < distance_eps || t[i] == 0) {//jesli droga jest mniejsza od najmniejszej wykrywalnej albo czas jest rowny 0
			if (t[i] == 0) {
				//printf("droga 0 (koncowe obliczenia) w osi %d\n", i);
				pose_list_iterator->przysp[i] = 0;
				pose_list_iterator->jedn[i] = 0;
				continue;
			}

			if (eq(pose_list_iterator->v_p[i], 0) && eq(
					pose_list_iterator->v_r[i], 0)) {
				t[i] = t_max; //unikniecie dalszej redukcji w sytuacji gdy v_p i v_r wynosza 0 (mozna wtedy dowolnie wydluzycz czas postoju bez redukcji predkosci)
			}

			if (fabs(t[i] - t_max) > 0) {//redukcja predkosci w danej osi ze wzgledu na zbyt krotki czas ruchu
				if (pose_list_iterator->model[i] == 1) { //kinematic_model_with_tool 1
					reduction_model_1(pose_list_iterator, i, s[i]);
					//printf("2 v_r: %f\n", pose_list_iterator->v_r[i]);
					if (trajectory_calculated == true) {//wyjscie z rekurencji
						return;
					}
				} else if (pose_list_iterator->model[i] == 2) {//kinematic_model_with_tool 2
					reduction_model_2(pose_list_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else if (pose_list_iterator->model[i] == 3) {//kinematic_model_with_tool 3
					reduction_model_3(pose_list_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else if (pose_list_iterator->model[i] == 4) {//kinematic_model_with_tool 4
					reduction_model_4(pose_list_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else {
					//printf(" ten przypadek nie moze wystapic (redukcja ze wzgledu na czas)\n");
					sr_ecp_msg.message(
							"Unexpected calculation error 2. Save your trajectory and report bug");
					throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
				}

				//printf("redukcja predkosci z powodu czasu w osi %d\n", i);
			}
			printf("v_p: %f\t v_r: %f\t v_r_next: %f\t a_r: %f\t v_k: %f\n",
					pose_list_iterator->v_p[i], pose_list_iterator->v_r[i],
					v_r_next[i], pose_list_iterator->a_r[i],
					pose_list_iterator->v_k[i]);
			pose_list_iterator->przysp[i] = fabs((pose_list_iterator->v_r[i]
					- pose_list_iterator->v_p[i]) / (pose_list_iterator->a_r[i]
					* tk));//zapisanie makrokroku w ktorym konczy sie przyspieszanie
			pose_list_iterator->jedn[i] = (t_max - (fabs(
					pose_list_iterator->v_r[i] - pose_list_iterator->v_k[i])
					/ pose_list_iterator->a_r[i])) / tk
					- pose_list_iterator->przysp[i];//zapisanie makrokroku w ktorym konczy sie jednostajny
			printf("przysp: %f\t jedn: %f\n", pose_list_iterator->przysp[i],
					pose_list_iterator->jedn[i]);
			//printf("jedn: %f\t t max: %f\t v_r: %f\t v_k: %f\t a_r: %f\t tk: %f\n",pose_list_iterator->jedn[i], t_max, pose_list_iterator->v_r[i], pose_list_iterator->v_k[i], pose_list_iterator->a_r[i], tk);
		}

		//obliczanie v_grip
		pose_list_iterator->v_grip = (s[gripp] / pose_list_iterator->t);

		switch (td.arm_type) {

		case lib::ECP_XYZ_EULER_ZYZ:
			if (pose_list_iterator->v_grip < v_grip_min_zyz) {
				pose_list_iterator->v_grip = v_grip_min_zyz;
			}
			break;

		case lib::ECP_XYZ_ANGLE_AXIS:
			if (pose_list_iterator->v_grip < v_grip_min_aa) {
				pose_list_iterator->v_grip = v_grip_min_aa;
				printf("ustawienie v_gripp na minimum\n");
			}
			break;

		case lib::ECP_MOTOR:
			if (pose_list_iterator->v_grip < v_grip_min_motor) {
				pose_list_iterator->v_grip = v_grip_min_motor;
			}
			break;

		case lib::ECP_JOINT:
			if (pose_list_iterator->v_grip < v_grip_min_joint) {
				pose_list_iterator->v_grip = v_grip_min_joint;
			}
			break;
		default:
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}

		//if (debug) {
		for (int os = 0; os < MAX_SERVOS_NR; os++) {
			printf(
					"\n=============== pozycja trajektorii nr %d pos: %d ============== os: %d ====\n",
					j, pose_list_iterator->pos_num, os);
			printf("czas ruchu %f\t model: %d\n", pose_list_iterator->t,
					pose_list_iterator->model[i]);
			printf("coordinates: %f\n", pose_list_iterator->coordinates[os]);
			printf("jedn: %f\t przysp: %f\n", pose_list_iterator->jedn[os],
					pose_list_iterator->przysp[os]);
			printf("liczba makrokrokow: %d\n",
					pose_list_iterator->interpolation_node_no);
			printf("start pos: %f\t kierunek (k): %f\n",
					pose_list_iterator->start_position[os],
					pose_list_iterator->k[os]);
			printf("s: %f\ns_przysp: %f\t s_jedn: %f\n", s[os],
					pose_list_iterator->s_przysp[os],
					pose_list_iterator->s_jedn[os]);
			printf("czas\t\tv_r\t\tv_r_next\ta_r\t\tv_p\t\tv_k\n");
			printf("%f\t%f\t%f\t%f\t%f\t%f\n", t[os],
					pose_list_iterator->v_r[os], v_r_next[os],
					pose_list_iterator->a_r[os], pose_list_iterator->v_p[os],
					pose_list_iterator->v_k[os]);
			printf("v_grip: %f\n\n", pose_list_iterator->v_grip);
			flushall();
		}

		printf(
				"************************************************** nowa pozycja **************************************************\n");
		printf("type: %d\n", type);
		flushall();
		//}
	}
	trajectory_calculated = true;

}//end - calculate

void smooth::reduction_model_1(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s) {
	//printf("redukcja kinematic_model_with_tool 1 w osi: %d\n", i);
	if (pose_list_iterator->v_p[i] < pose_list_iterator->v_k[i]
			&& (pose_list_iterator->v_k[i] * pose_list_iterator->t
					- 0.5 * ((pose_list_iterator->v_k[i]
							- pose_list_iterator->v_p[i])
							* (pose_list_iterator->v_k[i]
									- pose_list_iterator->v_p[i]))
							/ pose_list_iterator->a_r[i]) > s) {//proba dopasowania do modelu 2

		pose_list_iterator->model[i] = 2;
		//printf("dopasowanie kinematic_model_with_tool 2 w redukcji modelu 1\n");
		reduction_model_2(pose_list_iterator, i, s);

	} else if (pose_list_iterator->v_p[i] > pose_list_iterator->v_k[i]
			&& (pose_list_iterator->v_p[i] * pose_list_iterator->t
					- 0.5 * ((pose_list_iterator->v_p[i]
							- pose_list_iterator->v_k[i])
							* (pose_list_iterator->v_p[i]
									- pose_list_iterator->v_k[i]))
							/ pose_list_iterator->a_r[i]) > s) { // proba dopasowania do modelu 4

		pose_list_iterator->model[i] = 4;
		//printf("dopasowanie kinematic_model_with_tool 4 w redukcji modelu 1\n");
		reduction_model_4(pose_list_iterator, i, s);

	} else if (eq(pose_list_iterator->v_p[i], pose_list_iterator->v_k[i])
			&& pose_list_iterator->v_k[i] * pose_list_iterator->t > s) {//proba dopasowanie do modelu 3
		//printf("dopasowanie kinematic_model_with_tool 3 w redukcji modelu 1\n");
		reduction_model_3(pose_list_iterator, i, s);

	} else { //normalna redukcja dla modelu 1
		//printf("normalna redukcja kinematic_model_with_tool 1\n");

		double t1;//czas przyspieszania
		double t2;//czas jednostajnego
		double delta;//delta w rownaniu kwadratowym

		delta = (2 * pose_list_iterator->a_r[i] * pose_list_iterator->t + 2
				* pose_list_iterator->v_k[i] + 2 * pose_list_iterator->v_p[i])
				* (2 * pose_list_iterator->a_r[i] * pose_list_iterator->t + 2
						* pose_list_iterator->v_k[i] + 2
						* pose_list_iterator->v_p[i]) + 8
				* (-pose_list_iterator->v_p[i] * pose_list_iterator->v_p[i]
						- pose_list_iterator->v_k[i]
								* pose_list_iterator->v_k[i] - 2
						* pose_list_iterator->a_r[i] * s);

		//printf("delta: %f\n", delta);

		//printf("t: %f\t a_r: %f\t v_p: %f\n",pose_list_iterator->t, pose_list_iterator->a_r[i],pose_list_iterator->v_p[i]);

		if (!eq(delta, 0.0) && !eq(delta, -0.0)) {
			pose_list_iterator->v_r[i] = (-(2 * pose_list_iterator->a_r[i]
					* pose_list_iterator->t + 2 * pose_list_iterator->v_k[i]
					+ 2 * pose_list_iterator->v_p[i]) + sqrt(delta)) / (-4);
		} else {
			pose_list_iterator->v_r[i] = (-(2 * pose_list_iterator->a_r[i]
					* pose_list_iterator->t + 2 * pose_list_iterator->v_k[i]
					+ 2 * pose_list_iterator->v_p[i])) / (-4);
		}

		printf("v_r: %f\n", pose_list_iterator->v_r[i]);
		t1 = fabs(pose_list_iterator->v_p[i] - pose_list_iterator->v_r[i])
				/ pose_list_iterator->a_r[i];
		t2 = pose_list_iterator->t - t1 - (fabs(pose_list_iterator->v_k[i]
				- pose_list_iterator->v_r[i]) / pose_list_iterator->a_r[i]);

		//printf("t2: %f\t t1: %f\n", t2, t1);

		pose_list_iterator->s_przysp[i] = t1 * pose_list_iterator->v_p[i] + 0.5
				* pose_list_iterator->a_r[i] * t1 * t1;
		pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * t2;
	}
}

void smooth::reduction_model_2(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s) {
	//printf("redukcja kinematic_model_with_tool 2 w osi: %d\n", i);
	//pierwszy stopien redukcji
	double a;

	//printf("v_r: %f\t v_k: %f\t v_p: %f\t t: %f\n", pose_list_iterator->v_r[i], pose_list_iterator->v_k[i], pose_list_iterator->v_p[i], pose_list_iterator->t);

	a = (0.5 * (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])
			* (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])
			+ pose_list_iterator->v_p[i] * (pose_list_iterator->v_k[i]
					- pose_list_iterator->v_p[i]) - pose_list_iterator->v_k[i]
			* (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])) / (s
			- pose_list_iterator->v_k[i] * pose_list_iterator->t);

	if ((pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i]) / a
			> pose_list_iterator->t) { //drugi stopien redukcji
		if (s == pose_list_iterator->v_p[i] * pose_list_iterator->t) { //zabezpieczenie przed dzieleniem przez 0
			a = -1; //powoduje przejscie do nastepnego stopnia redukcji (moglaby byc dowolna ujemna liczba)
		} else {
			a
					= (0.5 * (pose_list_iterator->v_k[i]
							- pose_list_iterator->v_p[i])
							* (pose_list_iterator->v_k[i]
									- pose_list_iterator->v_p[i])) / (s
							- pose_list_iterator->v_p[i]
									* pose_list_iterator->t);
		}
		//printf("drugi stopien\n");
		if (a > pose_list_iterator->a_r[i] || a <= 0) {//trzeci stopien redukcji
			//printf("trzeci stopien\n");
			double t1; //czas konca opoznienia
			double s1; // droga w etapie w ktorym redukujemy czas (przed etapem "odcinanym")
			double t2; //czas redukowanego kawalka

			t2 = pose_list_iterator->t - ((pose_list_iterator->v_k[i]
					- pose_list_iterator->v_p[i]) / pose_list_iterator->a_r[i]);

			s1 = s - (pose_list_iterator->v_p[i] * (pose_list_iterator->t - t2)
					+ 0.5 * pose_list_iterator->a_r[i] * (pose_list_iterator->t
							- t2) * (pose_list_iterator->t - t2));

			if (pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i] * t2
					* t2 //ujemna liczba pod pierwiastkiem, zabezpieczenie
					- 4 * pose_list_iterator->a_r[i]
							* (pose_list_iterator->v_p[i] * t2 - s1) < 0) {
				vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
				return;
			}

			t1 = (pose_list_iterator->a_r[i] * t2 - (sqrt(
					pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i]
							* t2 * t2 - 4 * pose_list_iterator->a_r[i]
							* (pose_list_iterator->v_p[i] * t2 - s1)))) / (2
					* pose_list_iterator->a_r[i]);

			if (pose_list_iterator->v_p[i] - pose_list_iterator->a_r[i] * t1
					< 0) {//ujemna predkosc ruchu, zabezpieczenie
				vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
				return;
			}

			pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i]
					- pose_list_iterator->a_r[i] * t1;
			pose_list_iterator->s_przysp[i] = 0.5 * pose_list_iterator->a_r[i]
					* t1 * t1 + pose_list_iterator->v_r[i] * t1;
			pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * (t2
					- 2 * t1);

			return;
		}

		pose_list_iterator->a_r[i] = a;
		pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i];
		pose_list_iterator->s_przysp[i] = 0;
		pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i]
				* (pose_list_iterator->t - (pose_list_iterator->v_k[i]
						- pose_list_iterator->v_p[i])
						/ pose_list_iterator->a_r[i]);

		return;
	}

	pose_list_iterator->a_r[i] = a;
	//printf("zredukowane a_r: %f\n", pose_list_iterator->a_r[i]);
	pose_list_iterator->v_r[i] = pose_list_iterator->v_k[i];
	pose_list_iterator->s_przysp[i] = pose_list_iterator->v_p[i]
			* (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])
			/ pose_list_iterator->a_r[i] + 0.5 * (pose_list_iterator->v_k[i]
			- pose_list_iterator->v_p[i]) * (pose_list_iterator->v_k[i]
			- pose_list_iterator->v_p[i]) / pose_list_iterator->a_r[i];
	pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i]
			* (pose_list_iterator->t - (pose_list_iterator->v_k[i]
					- pose_list_iterator->v_p[i]) / pose_list_iterator->a_r[i]);
}

void smooth::reduction_model_3(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s) {
	//printf("redukcja kinematic_model_with_tool 3 w osi: %d\n", i);
	double t1; //czas konca opoznienia

	if (pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i]
			* pose_list_iterator->t * pose_list_iterator->t//liczba pierwiastkowana mniejsza od 0, zabezpieczenie
			- 4 * pose_list_iterator->a_r[i] * (pose_list_iterator->v_p[i]
					* pose_list_iterator->t - s) < 0) {
		vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
		return;
	}

	t1 = (pose_list_iterator->a_r[i] * pose_list_iterator->t - (sqrt(
			pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i]
					* pose_list_iterator->t * pose_list_iterator->t - 4
					* pose_list_iterator->a_r[i] * (pose_list_iterator->v_p[i]
					* pose_list_iterator->t - s)))) / (2
			* pose_list_iterator->a_r[i]);

	if (pose_list_iterator->v_p[i] - pose_list_iterator->a_r[i] * t1 < 0) {//ujemna predkosc ruchu, zabezpieczenie
		vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
		return;
	}

	pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i]
			- pose_list_iterator->a_r[i] * t1;
	pose_list_iterator->s_przysp[i] = 0.5 * pose_list_iterator->a_r[i] * t1
			* t1 + pose_list_iterator->v_r[i] * t1;
	pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i]
			* (pose_list_iterator->t - 2 * t1);
}

void smooth::reduction_model_4(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s) {
	//printf("redukcja kinematic_model_with_tool 4 w osi: %d\n", i);
	//pierwszy stopien redukcji
	double a;

	a = (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])
			* (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) / ((-2)
			* (s - pose_list_iterator->v_p[i] * pose_list_iterator->t));

	if ((pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i]) / a
			> pose_list_iterator->t) { //drugi stopien redukcji
		if (s == pose_list_iterator->v_k[i] * pose_list_iterator->t) { //zabezpieczenie przed dzieleniem przez 0
			a = -1; //powoduje przejscie do nastepnego stopnia redukcji (moglaby byc dowolna ujemna liczba)
		} else {
			a
					= (0.5 * (pose_list_iterator->v_p[i]
							- pose_list_iterator->v_k[i])
							* (pose_list_iterator->v_p[i]
									- pose_list_iterator->v_k[i])) / (s
							- pose_list_iterator->v_k[i]
									* pose_list_iterator->t);
		}
		//printf("drugi stopien\n");
		if (a > pose_list_iterator->a_r[i] || a <= 0) {//trzeci stopien redukcji
			double t1; //czas konca opoznienia (relatywny - liczac od poczatku redukowanego odcinka)
			double s1; // droga w etapie w ktorym redukujemy czas (po etapie odcinanym)
			double t2; //czas redukowanego kawalka (relatywny)
			//printf("trzeci stopien");

			t2 = pose_list_iterator->t - ((pose_list_iterator->v_p[i]
					- pose_list_iterator->v_k[i]) / pose_list_iterator->a_r[i]);

			s1 = s - (pose_list_iterator->v_k[i] * (pose_list_iterator->t - t2)
					+ 0.5 * pose_list_iterator->a_r[i] * (pose_list_iterator->t
							- t2) * (pose_list_iterator->t - t2));

			if (pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i] * t2
					* t2 //liczba pod pierwiastkiem mniejsza od 0, zabezpieczenie
					- 4 * pose_list_iterator->a_r[i]
							* (pose_list_iterator->v_k[i] * t2 - s1) < 0) {
				vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
				return;
			}

			t1 = (pose_list_iterator->a_r[i] * t2 - (sqrt(
					pose_list_iterator->a_r[i] * pose_list_iterator->a_r[i]
							* t2 * t2 - 4 * pose_list_iterator->a_r[i]
							* (pose_list_iterator->v_k[i] * t2 - s1)))) / (2
					* pose_list_iterator->a_r[i]);

			if ((pose_list_iterator->v_k[i] - pose_list_iterator->a_r[i] * t1)
					< 0) {//ujemna predkosc ruchu, zabezpieczenie
				vp_reduction(pose_list_iterator, i, s, pose_list_iterator->t);
				return;
			}

			pose_list_iterator->v_r[i] = pose_list_iterator->v_k[i]
					- pose_list_iterator->a_r[i] * t1;
			pose_list_iterator->s_przysp[i] = 0.5 * pose_list_iterator->a_r[i]
					* t1 * t1 + pose_list_iterator->v_r[i] * t1
					+ pose_list_iterator->v_k[i] * (pose_list_iterator->t - t2)
					+ 0.5 * pose_list_iterator->a_r[i] * (pose_list_iterator->t
							- t2) * (pose_list_iterator->t - t2);
			pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i] * (t2
					- 2 * t1);

			return;
		}

		pose_list_iterator->a_r[i] = a;
		//printf("a_r: %f", pose_list_iterator->v_p[i]);
		pose_list_iterator->v_r[i] = pose_list_iterator->v_k[i];
		pose_list_iterator->s_przysp[i] = pose_list_iterator->v_r[i]
				* (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])
				/ pose_list_iterator->a_r[i] + 0.5
				* (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])
				* (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])
				/ pose_list_iterator->a_r[i];
		pose_list_iterator->s_jedn[i] = pose_list_iterator->v_r[i]
				* (pose_list_iterator->t - (pose_list_iterator->v_p[i]
						- pose_list_iterator->v_k[i])
						/ pose_list_iterator->a_r[i]);

		return;
	}

	pose_list_iterator->a_r[i] = a;
	pose_list_iterator->v_r[i] = pose_list_iterator->v_p[i];
	pose_list_iterator->s_przysp[i] = 0;
	pose_list_iterator->s_jedn[i] = pose_list_iterator->v_p[i]
			* (pose_list_iterator->t - (pose_list_iterator->v_p[i]
					- pose_list_iterator->v_k[i]) / pose_list_iterator->a_r[i]);
}

void smooth::vp_reduction(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s, double t) {
	printf("v_p redukcja w osi: %d\n", i);
	double v_r; //zmiana ruchu na jednostajny

	v_r = s / t;

	switch (td.arm_type) {//zapisanie nowej pr��dko��ci w liscie pozycji, dla danej pozycji

	case lib::ECP_XYZ_EULER_ZYZ:
		pose_list_iterator->v[i] = v_r / v_max_zyz[i];
		break;

	case lib::ECP_XYZ_ANGLE_AXIS:
		pose_list_iterator->v[i] = v_r / v_max_aa[i];
		break;

	case lib::ECP_MOTOR:
		pose_list_iterator->v[i] = v_r / v_max_motor[i];
		break;

	case lib::ECP_JOINT:
		pose_list_iterator->v[i] = v_r / v_max_zyz[i];
		break;
	default:
		throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}

	calculate();
}

void smooth::vk_reduction(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s, double t) {
	//printf("v_k redukcja w osi: %d\n", i);
	double a;
	double v_k;

	a = (2 * (s - (pose_list_iterator->v_p[i] * t))) / (t * t);

	if (a < 0) {
		//printf("v_k stopien 2\n");
		a = (-2 * s + 2 * t * pose_list_iterator->v_p[i]) / (t * t);
		v_k = (-1) * a * t + pose_list_iterator->v_p[i];
		if (eq(v_k, -0.0)) {//glupie... ale inaczej nie dziala gdy v_k jest rowne 0
			v_k = 0.0;
		}

		if (a > pose_list_iterator->a_r[i] || v_k < 0 || v_k
				> pose_list_iterator->v_k[i]) {
			//printf("v_k: %f\n", v_k);
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
	pose_list_iterator->v_r[i] = pose_list_iterator->v_k[i];
	pose_list_iterator->s_przysp[i] = s;
	pose_list_iterator->s_jedn[i] = 0;
}

void smooth::optimize_time1(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s) {
	printf("\noptymalizacja czasu 1 os: %d\n", i);
	double v_r;
	double t;

	v_r = sqrt(pose_list_iterator->a_r[i] * s + (pose_list_iterator->v_p[i]
			* pose_list_iterator->v_p[i]) / 2 + (pose_list_iterator->v_k[i]
			* pose_list_iterator->v_k[i]) / 2);
	//pose_list_iterator->t = t;

	if (pose_list_iterator->v_p[i] >= pose_list_iterator->v_k[i] && v_r
			< pose_list_iterator->v_p[i]) {
		t = (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])
				/ pose_list_iterator->a_r[i];
		printf("vp reduction\n");
		vp_reduction(pose_list_iterator, i, s, t);
		return;
	} else if (pose_list_iterator->v_p[i] < pose_list_iterator->v_k[i] && v_r
			< pose_list_iterator->v_k[i]) {
		t = (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])
				/ pose_list_iterator->a_r[i];
		pose_list_iterator->v_r[i] = pose_list_iterator->v_k[i];
		printf("vk reduction\n");
		vk_reduction(pose_list_iterator, i, s, t);
		return;
	} else {
		printf("normal\n");
		t = ((v_r - pose_list_iterator->v_p[i]) / pose_list_iterator->a_r[i]
				+ (v_r - pose_list_iterator->v_k[i])
						/ pose_list_iterator->a_r[i]);
		pose_list_iterator->v_r[i] = v_r;
	}
	pose_list_iterator->t = t;
	//printf("v_r: %f\t t: %f\n", v_r, t);
}

void smooth::optimize_time2(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s) {
	printf("\noptymalizacja czasu 2 os: %d\n", i);
	double v_r;
	double t;

	v_r = sqrt(2 * pose_list_iterator->a_r[i] * s + pose_list_iterator->v_p[i]
			* pose_list_iterator->v_p[i]);
	pose_list_iterator->v_k[i] = v_r;
	t = (pose_list_iterator->v_k[i] - pose_list_iterator->v_p[i])
			/ pose_list_iterator->a_r[i];

	pose_list_iterator->t = t;
	printf("v_r: %f\t t: %f\n", v_r, t);
}

void smooth::optimize_time4(
		std::list<ecp_mp::common::smooth_trajectory_pose>::iterator pose_list_iterator,
		int i, double s) {
	printf("\noptymalizacja czasu 4 os: %d\n", i);
	double v_r;
	double t;

	v_r = sqrt(2 * pose_list_iterator->a_r[i] * s + pose_list_iterator->v_k[i]
			* pose_list_iterator->v_k[i]);
	pose_list_iterator->v_p[i] = v_r; //niepotrzebne?
	t = (pose_list_iterator->v_p[i] - pose_list_iterator->v_k[i])
			/ pose_list_iterator->a_r[i];

	pose_list_iterator->t = t;
	printf("v_r: %f\t t: %f\n", v_r, t);
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
