/**
 * \file ecp_g_newsmooth.cc
 * \brief newsmooth class and its methods
 *
 * Contains bodies of the methods of newsmooth class.
 */

#include "generator/ecp/ecp_g_newsmooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

void newsmooth::reset() {

}

void newsmooth::load_a_v_max_from_file(const char* file_name)
{
    /*std::ifstream from_file(file_name); // otworz plik do odczytu

    if (!from_file.good())
    {
        perror(file_name);
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    for (int j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_motor[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for (int j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_motor[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for (int j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_joint[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for (int j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_joint[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for (int j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_zyz[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for (int j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_zyz[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for (int j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_aa[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for (int j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_aa[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }*/
} // end: bool load_a_v_max()

newsmooth::newsmooth (common::task::task& _ecp_task, bool _is_synchronised, bool _debug) :
		multiple_position<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose> (_ecp_task)

{
	//pose_vector = vector<ecp_mp::common::trajectory_pose::trajectory_pose>();
	//coordinate_vector = vector<vector<double> >();
} // end : konstruktor

/*double newsmooth::generate_next_coords(int node_counter, int interpolation_node_no, double start_position, double v_p, double v_r,
									double v_k, double a_r, int k, double przysp, double jedn, double s_przysp, double s_jedn) {

    //funkcja obliczajaca polozenie w danym makrokroku

	double next_position;

    double tk=10*STEP;

	if(node_counter < przysp+1) { //pierwszy etap
		if(v_p <= v_r) { //przyspieszanie w pierwszym etapie
			if (type == lib::ABSOLUTE) {//tryb absolute

				if (przysp > (node_counter-1) && przysp < (node_counter)) {//przyspieszanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku

						next_position = start_position + k * (przysp*v_p*tk + przysp*przysp*a_r*tk*tk/2 + jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k* (v_r * (node_counter - przysp - jedn)*tk - tk*tk*(node_counter - jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						} else {//przyspiesznie w trzecim etapie
							next_position += k* (v_r * (node_counter - przysp - jedn)*tk + tk*tk*(node_counter - jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						}
					} else {//przyspieszenie + poczatek jednostajnego
						next_position = start_position + k * (v_p * node_counter * tk + przysp * przysp * tk * tk * a_r / 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne przyspieszanie
					next_position = start_position + k*(node_counter*v_p*tk + node_counter*node_counter*a_r*tk*tk/2);
				}

			} else if (type == lib::RELATIVE) {//tryb relatywny
				if (przysp > (node_counter-1) && przysp < (node_counter)) {//przyspieszanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku

						next_position = k * ((1 - node_counter + przysp)*v_r*tk - (1 - node_counter + przysp)*(1 - node_counter + przysp)*a_r*tk*tk/2 + jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k* (v_r * (node_counter - przysp - jedn)*tk - tk*tk*(node_counter - jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						} else {//przyspiesznie w trzecim etapie
							next_position += k* (v_r * (node_counter - przysp - jedn)*tk + tk*tk*(node_counter - jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						}
					} else {//przyspieszenie + poczatek jednostajnego
						next_position = k * (v_r * (1 - node_counter + przysp) * tk - (1 - node_counter + przysp) * (1 - node_counter + przysp) * tk * tk * a_r / 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne przyspieszanie
					next_position = k * (v_p * tk + (node_counter - 1) * a_r * tk * tk + (a_r * tk * tk)/2);
				}
			}

		} else { //hamowanie w pierwszym etapie
			if (type == lib::ABSOLUTE) {
				if ((przysp) > node_counter && (przysp) < (node_counter+1)) {

					if (przysp + jedn < node_counter) {

						next_position = start_position + k * (przysp*v_p*tk - przysp*przysp*a_r*tk*tk/2 + jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k* (v_r * (node_counter - przysp - jedn)*tk - tk*tk*(node_counter - jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						} else {//przyspieszanie w trzecim etapie
							next_position += k* (v_r * (node_counter - przysp - jedn)*tk + tk*tk*(node_counter - jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						}
					} else {//hamowanie + poczatek jednostajnego
						next_position = start_position + k * (v_p * node_counter * tk - przysp * przysp * tk * tk * a_r / 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne hamowanie
					next_position = start_position + k*(node_counter*tk*v_p - node_counter*node_counter*tk*tk*a_r/2);
				}

			} else if (type == lib::RELATIVE) {
				if (przysp > (node_counter-1) && przysp < (node_counter)) {//hamowanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku
						next_position = k * ((1 - node_counter + przysp)*v_r*tk + (1 - node_counter + przysp)*(1 - node_counter + przysp)*a_r*tk*tk/2 + jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k* (v_r * (node_counter - przysp - jedn)*tk - tk*tk*(node_counter - jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						} else {//przyspieszanie w trzecim etapie
							next_position += k* (v_r * (node_counter - przysp - jedn)*tk + tk*tk*(node_counter - jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						}
					} else {//hamowanie + poczatek jednostajnego
						next_position = k * (v_r * (1 - node_counter + przysp) * tk + (1 - node_counter + przysp) * (1 - node_counter + przysp) * tk * tk * a_r / 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne hamowanie
					next_position = k * (v_p * tk - ((node_counter - 1) * a_r * tk * tk + (a_r * tk * tk)/2));
				}
			}
		}
	} else if(node_counter <= przysp + jedn + 1) { // drugi etap - ruch jednostajny
		if (type == lib::ABSOLUTE) {
			if ((przysp+jedn) > (node_counter-1) && (przysp+jedn) < (node_counter)) {//jednostajny wchodzi w faze trzecia

				next_position = start_position + k*(s_przysp + s_jedn);

				if (v_r > v_k) {//hamowanie w 3 etapie
					next_position += k * (v_r * (node_counter - jedn - przysp) * tk - (node_counter - jedn - przysp) * (node_counter - jedn - przysp) * tk * tk * a_r / 2);
				} else {//przyspieszanie w 3 etapie
					next_position += k * (v_r * (node_counter - jedn - przysp) * tk + (node_counter - jedn - przysp) * (node_counter - jedn - przysp) * tk * tk * a_r / 2);
				}
			} else { //normalny jednostajny
				next_position = start_position + k * (s_przysp + ((node_counter - przysp)*tk)*v_r);
			}
		} else if (type == lib::RELATIVE) {
			if ((przysp+jedn) > (node_counter-1) && (przysp+jedn) < (node_counter)) {//jednostajny wchodzi w faze trzecia

				next_position = k* (v_r * tk * (1 - node_counter + przysp + jedn));

				if (v_r > v_k) {//hamowanie w 3 etapie
					next_position += k * (v_r * (node_counter - jedn - przysp) * tk - (node_counter - jedn - przysp) * (node_counter - jedn - przysp) * tk * tk * a_r / 2);
				} else {//przyspieszanie w 3 etapie
					next_position += k * (v_r * (node_counter - jedn - przysp) * tk + (node_counter - jedn - przysp) * (node_counter - jedn - przysp) * tk * tk * a_r / 2);
				}
			} else { //normalny jednostajny
				next_position = k * v_r * tk;
			}
		}

	} else if(node_counter <= interpolation_node_no) { //trzeci etap

		if(v_k <= v_r) { //hamowanie w trzecim etapie
			if (type == lib::ABSOLUTE) {
				next_position = start_position +
								   k*(s_przysp + s_jedn + ((node_counter - jedn - przysp) * tk ) * v_r -
										 ((node_counter - jedn - przysp) * tk) * ((node_counter - jedn - przysp) * tk) * a_r / 2);
			} else if (type == lib::RELATIVE) {
				next_position = k * ((v_k + (interpolation_node_no - node_counter) * a_r * tk) * tk + (a_r * tk * tk)/2);
			}
		} else { //przyspieszanie w trzecim etapie
			if (type == lib::ABSOLUTE) {
				next_position = start_position +
								   k*(s_przysp + s_jedn +
										 ((node_counter - jedn - przysp) * tk) * v_r +
										 a_r * ((node_counter - jedn - przysp) * tk) * ((node_counter - jedn - przysp) * tk) / 2);
			} else if (type == lib::RELATIVE) {
				next_position = k* ((v_k - (interpolation_node_no - node_counter) * a_r * tk) * tk - (a_r * tk * tk)/2);
			}
		}
	}



    return next_position;
}

void newsmooth::generate_coords() {

	double coordinate[MAX_SERVOS_NR];

	double general_temp[MAX_SERVOS_NR];
	double prev_coordinate[MAX_SERVOS_NR];
	double coordinate_backup[MAX_SERVOS_NR];
	lib::Homog_matrix goal_frame;
	int g;
	int h;
	int f;

	int private_node_counter = 1;
	initiate_pose_list();
	flush_coordinate_list();
	for (int j = 0; j < pose_list_length(); j++) {
		for (h = 0; h < MAX_SERVOS_NR; h++) {
			prev_coordinate[h] = pose_vector_iterator->start_position[h];
		}

		lib::Homog_matrix begining_frame;
		begining_frame.set_from_xyz_angle_axis(prev_coordinate);
		cout << begining_frame << endl;

		goal_frame.set_from_xyz_angle_axis(prev_coordinate);
		//printf("start pos w generate_cords: %f\t w osi: %d\n",pose_list_iterator->start_position[0], 0);

		for (int z = 0; z < pose_vector_iterator->interpolation_node_no; z++) {
			for (int i = 0; i < MAX_SERVOS_NR; i++) {
				if (type == lib::ABSOLUTE && fabs(pose_vector_iterator->start_position[i] - pose_vector_iterator->coordinates[i]) < distance_eps) {
					coordinate[i] = pose_vector_iterator->start_position[i]; //dla drogi 0
				} else if (type == lib::RELATIVE && eq(pose_vector_iterator->coordinates[i], 0)) {//dla drogi 0 w relative
					coordinate[i] = 0;
				} else {
					coordinate[i] = generate_next_coords(private_node_counter,
							pose_vector_iterator->interpolation_node_no,
							pose_vector_iterator->start_position[i],
							pose_vector_iterator->v_p[i],
							pose_vector_iterator->v_r[i],
							pose_vector_iterator->v_k[i],
							pose_vector_iterator->a_r[i],
							pose_vector_iterator->k[i],
							pose_vector_iterator->przysp[i],
							pose_vector_iterator->jedn[i],
							pose_vector_iterator->s_przysp[i],
							pose_vector_iterator->s_jedn[i]);
				}
			}
			private_node_counter++;
			//coordinate_list.push_back(coordinates(coordinate));
			for (h = 0; h < MAX_SERVOS_NR; h++) {
				coordinate_backup[h] = coordinate[h];
			}

			lib::Homog_matrix begining_frame_with_current_translation = begining_frame;
			begining_frame_with_current_translation.set_translation_vector(goal_frame);

			lib::Xyz_Angle_Axis_vector step_of_total_increment_vector =
						lib::V_tr(!(lib::V_tr(!begining_frame_with_current_translation
								* goal_frame))) * lib::Xyz_Angle_Axis_vector(coordinate_backup);

			goal_frame = goal_frame * lib::Homog_matrix(step_of_total_increment_vector);

			lib::Xyz_Angle_Axis_vector tmp_angle_axis_vector;
			goal_frame.get_xyz_angle_axis(tmp_angle_axis_vector);
			tmp_angle_axis_vector.to_table(general_temp);

			coordinate_list.push_back(coordinates(general_temp));

			for(g = 0; g < 6; g++) {
				printf("%f\t", general_temp[g]);
			}
			for(g = 0; g < 3; g++) {
				printf("%f\t", coordinate[g]);
			}
			printf("\n");
			flushall();
			//end: testowo
		}
		private_node_counter = 1;
		next_pose_list_ptr();
	}

	trajectory_generated = true;
	//printowanie listy coordinate
	initiate_coordinate_list();
	for (int m = 0; m < coordinate_list.size(); m++) {
		//printf("makrokrok: %d\t", m);
		for (int n = 0; n < 8; n++) {
			//printf("%f\t", coordinate_list_iterator->coordinate[n]);
		}
		//printf("\n");
		flushall();
		coordinate_list_iterator++;
	}
	//printf("\ngenerate_cords\n");
}

void newsmooth::send_coordinates() {
	//lib::Xyz_Euler_Zyz_vector tmp_euler_vector;
    double gripper_position;
    double tk=10*STEP;
	int i; //licznik petli
	int gripp; //os grippera

	the_robot->ecp_command.instruction.instruction_type = lib::SET; //ustawienie parametrow ruchu w edp_data
	the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION; //ponizej w caseach jest dalsze ustawianie
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	switch (td.arm_type) {

		case lib::ECP_XYZ_EULER_ZYZ:

			homog_matrix.set_from_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector(coordinate_list_iterator->coordinate));
			homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

			gripp = 6;

			break;

		case lib::ECP_XYZ_ANGLE_AXIS:

			homog_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(coordinate_list_iterator->coordinate));
			homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

			gripp = 6;

			break;

		case lib::ECP_JOINT:

			for (i = 0; i < MAX_SERVOS_NR; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = coordinate_list_iterator->coordinate[i];
			}

			if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
				gripp = 7;
			} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
				gripp = 6;
			}

			break;

		case lib::ECP_MOTOR:

			for (i=0; i < MAX_SERVOS_NR; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = coordinate_list_iterator->coordinate[i];
			}

			if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
				gripp = 7;
			} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
				gripp = 6;
			}

			break;

		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch
	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;
	//gripper
	/*if (type == lib::RELATIVE && !external_absolute) {
		gripper_position = pose_list_iterator->k[gripp]*((tk)*pose_list_iterator->v_grip);

		//printf("gripper_position: %f\n ", gripper_position);

		//printf(" %f \t", gripper_position);
		if((node_counter * gripper_position > pose_list_iterator->coordinates[gripp] && pose_list_iterator->k[gripp] == -1) ||
		(node_counter * gripper_position < pose_list_iterator->coordinates[gripp] && pose_list_iterator->k[gripp] == 1)) {
		//printf("git");
			the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = gripper_position;
		} else {
			the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = 0;
		}
	} else {

		gripper_position = pose_list_iterator->start_position[gripp] +
										   pose_list_iterator->k[gripp]*((node_counter*tk)*pose_list_iterator->v_grip);
		//printf(" %f ", gripper_position);
		if((gripper_position > pose_list_iterator->coordinates[gripp] && pose_list_iterator->k[gripp] == -1) ||
				(gripper_position < pose_list_iterator->coordinates[gripp] && pose_list_iterator->k[gripp] == 1)) {
			//printf("git");
			the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = gripper_position;
		} else {
			printf("no gripp\t");
			the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = pose_list_iterator->coordinates[gripp];
		}
	}

	for (int n = 0; n < gripp; n++) {
		//printf("%f\t", coordinate_list_iterator->coordinate[n]);
	}

	//printf("%f\n", gripper_position);

	flushall();

	coordinate_list_iterator++;
}*/

//set necessary instructions, and other data for preparing the robot
bool newsmooth::first_step() { //wywolywane tylko raz w calej trajektorii

	/*//flush_coordinate_list();
	int gripp; //licznik petli
    initiate_pose_list();
    td.arm_type = pose_vector_iterator->arm_type;
    if (td.arm_type == lib::ECP_XYZ_ANGLE_AXIS && type == lib::ABSOLUTE) {
    	//set_relative();
    	external_absolute = true;
    } else {
    	external_absolute = false;
    }

    first_interval=true;//to chyba nie jest potrzebne bo ustawianie first_interval jest takze na poczatku calculate()

    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.instruction_type = lib::GET;

    switch ( td.arm_type ) {

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
        break;

    case lib::ECP_XYZ_EULER_ZYZ:
        the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
        the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
        if (type == lib::RELATIVE) {
            the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
            the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
            for (int i=0; i<6; i++)
            {
            	the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
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
    		for (int i=0; i<6; i++)
    		{
    			the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
    		}
    	} else {
    		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
    		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    	}
        break;

    default:
        throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    } // end : switch ( td.arm_type )
*/
    return true;
} // end: bool ecp_newsmooth_generator::first_step ( )

bool newsmooth::next_step () {

    /*if (!trajectory_generated && !trajectory_calculated) {
    	calculate(); //wypelnienie pozostalych zmiennych w liscie pose_list
    	generate_coords(); //wypelnienie listy coordinate_list (lista kolejnych pozycji w makrokrokach)
    	initiate_pose_list(); //ustawienie iteratora pose_list na poczatek
    	initiate_coordinate_list(); //ustawienie iteratora coordinate_list na poczatek
    }

    if (!trajectory_generated || !trajectory_calculated) {
    	return false;
    }

    if (node_counter == pose_vector_iterator->interpolation_node_no) {//czy poprzedni makrokrok byl ostatnim

    	if(is_last_list_element()) { //ostatni punkt (koniec listy pozycji pose_list)
    		//if (coordinate_list_itarator)
    		send_coordinates();
    		reset();
    		return false;

    	} else {//lista pozycji pose_list nie jest skonczona wiec idziemy do nastepnego punktu

    		send_coordinates();
			node_counter = 0; //TODO przestestowac, sprawdzic czy 1 czy 0
		    next_pose_list_ptr();
		    td.interpolation_node_no = pose_vector_iterator->interpolation_node_no;
		}
    } else {

    	send_coordinates();

    }// end: if*/
    return true;

} // end: bool ecp_newsmooth_generator::next_step ( )

/*void newsmooth::calculate(void) { //zeby wrocic do starego trybu relative nalezy stad usunac wszystkie warunki na type i zostawic opcje dla type = 1

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

		td.arm_type = pose_vector_iterator->arm_type;

		if (first_interval) {//pierwsza pozycja ruchu
			//wywoluje sie tylko raz, przy pierwzszym ruchu w trajektorii, musi byc rozroznione, gdyz tutaj v_p jest = 0 a pozycja jest
			// ...odczytywana z ramienia...

	    	pose_vector_iterator->pos_num = j;

			switch (td.arm_type) {

			case lib::ECP_XYZ_EULER_ZYZ:
				gripp = 6;
				homog_matrix.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);

				homog_matrix.get_xyz_euler_zyz(tmp_euler_vector);
				tmp_euler_vector.to_table(pose_vector_iterator->start_position);


				for (i = 0; i < gripp; i++) {
					temp = pose_vector_iterator->coordinates[i];

					if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_vector_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

				temp = pose_vector_iterator->coordinates[gripp];
				pose_vector_iterator->start_position[gripp] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
				if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_vector_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}

				pose_vector_iterator->start_position[gripp+1] = 0.0;

				for (i = 0; i < MAX_SERVOS_NR; i++) {

					if (v_max_zyz[i] == 0 || a_max_zyz[i] == 0 || pose_vector_iterator->v[i] == 0 || pose_vector_iterator->a[i] == 0) {
						sr_ecp_msg.message("One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
					}

					pose_vector_iterator->v_r[i] = v_max_zyz[i] * pose_vector_iterator->v[i];
					pose_vector_iterator->a_r[i] = a_max_zyz[i] * pose_vector_iterator->a[i];

					pose_vector_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0
					if (type == lib::ABSOLUTE) {
						if(pose_vector_iterator->coordinates[i]-pose_vector_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
							pose_vector_iterator->k[i] = -1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
						}
						else {
							pose_vector_iterator->k[i] = 1;
						}
					} else if (type == lib::RELATIVE) {
						if (pose_vector_iterator->coordinates[i] < 0) {
							pose_vector_iterator->k[i] = -1;
						} else {
							pose_vector_iterator->k[i] = 1;
						}
					}
				}

				break;

			case lib::ECP_XYZ_ANGLE_AXIS:
				gripp = 6;

				homog_matrix.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);

				homog_matrix.get_xyz_angle_axis(tmp_angle_axis_vector);
				tmp_angle_axis_vector.to_table(pose_vector_iterator->start_position);

				for (int k = 0; k < gripp; k++) {\
					printf("pozycja startowa %f\n", pose_vector_iterator->start_position[k]);
				}

				for (i = 0; i < gripp; i++) {
					temp = pose_vector_iterator->coordinates[i];
					if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_vector_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}


				if (external_absolute == true) {//w obecnej implementacji zawsze spelniony
					set_relative();
					homog_matrix.set_from_xyz_angle_axis(pose_vector_iterator->start_position);
					homog_matrix2.set_from_xyz_angle_axis(pose_vector_iterator->coordinates);

					((!homog_matrix) * homog_matrix2).get_xyz_angle_axis(tmp_angle_axis_vector);
					tmp_angle_axis_vector.to_table(pose_vector_iterator->coordinates);

				}

				//chwytak
				pose_vector_iterator->start_position[gripp] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
				temp = pose_vector_iterator->coordinates[gripp];

				if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_vector_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}

				pose_vector_iterator->coordinates[gripp] = pose_vector_iterator->coordinates[gripp] - pose_vector_iterator->start_position[gripp]; //nowa czesc
				pose_vector_iterator->start_position[gripp+1] = 0.0;

				for (i = 0; i < MAX_SERVOS_NR; i++) {

					if (v_max_aa[i] == 0 || a_max_aa[i] == 0 || pose_vector_iterator->v[i] == 0 || pose_vector_iterator->a[i] == 0) {//sprawdzenie czy zadna z predkosci ani przyspieszen nie jest 0
						sr_ecp_msg.message("One or more of 'v' or 'a' values is 0"); //TODO przerobic na mniejsze lub rowne
						throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
					}

					pose_vector_iterator->v_r[i] = v_max_aa[i] * pose_vector_iterator->v[i];
					pose_vector_iterator->a_r[i] = a_max_aa[i] * pose_vector_iterator->a[i];

					pose_vector_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) { //musi byc oddzielna petla bo gripper jest liczony w absolute a inne pozycje w relative
					if (type == lib::ABSOLUTE) { //ten warunek w nowej implementacji nigdy nie jest spelniony, angle axis i eulery dzialaja zawsze w relative
						if(pose_vector_iterator->coordinates[i]-pose_vector_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
							pose_vector_iterator->k[i] = -1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
						} else {
							pose_vector_iterator->k[i] = 1;
						}
					} else if (type == lib::RELATIVE) {
						if (pose_vector_iterator->coordinates[i] < 0) {
							pose_vector_iterator->k[i] = -1;
						} else {
							pose_vector_iterator->k[i] = 1;
						}
					}
				}

				break;

			case lib::ECP_JOINT:
				if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
					gripp=7;
				} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
					gripp=6;
				}

				for (i = 0; i < gripp; i++) {
					temp = pose_vector_iterator->coordinates[i];
					pose_vector_iterator->start_position[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];//pierwsze przypisanie start_position
					if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_vector_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}
				temp = pose_vector_iterator->coordinates[gripp];
				pose_vector_iterator->start_position[gripp] = the_robot->reply_package.arm.pf_def.arm_coordinates[gripp];
				if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_vector_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}
				if (gripp < (MAX_SERVOS_NR -1) ) {
					pose_vector_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if(!(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT && i == (MAX_SERVOS_NR - 1))) {
						if (v_max_joint[i] == 0 || a_max_joint[i] == 0 || pose_vector_iterator->v[i] == 0 || pose_vector_iterator->a[i] == 0) {
							sr_ecp_msg.message("One or more of 'v' or 'a' values is 0");
							throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
						}
					}

					pose_vector_iterator->v_r[i] = v_max_joint[i] * pose_vector_iterator->v[i];
					pose_vector_iterator->a_r[i] = a_max_joint[i] * pose_vector_iterator->a[i];

					pose_vector_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if (type == lib::ABSOLUTE) {
						if(pose_vector_iterator->coordinates[i]-pose_vector_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
							pose_vector_iterator->k[i] = -1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
						}
						else {
							pose_vector_iterator->k[i] = 1;
						}
					} else if (type == lib::RELATIVE) {
						if (pose_vector_iterator->coordinates[i] < 0) {
							pose_vector_iterator->k[i] = -1;
						} else {
							pose_vector_iterator->k[i] = 1;
						}
					}
				}

				break;

			case lib::ECP_MOTOR:
				if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
					gripp=7;
				} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
					gripp=6;
				}

				for (i = 0; i < gripp; i++) {
					temp = pose_vector_iterator->coordinates[i];
					pose_vector_iterator->start_position[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];//pierwsze przypisanie start_position
					if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
						next_pose_list_ptr();
						pose_vector_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}
				temp = pose_vector_iterator->coordinates[gripp];
				pose_vector_iterator->start_position[gripp] = the_robot->reply_package.arm.pf_def.arm_coordinates[gripp];
				if (!is_last_list_element()) {		   //musi byc zrobione tutaj zeby zadzialalo przypisanie kierunkow dla drugiego ruchu
					next_pose_list_ptr();
					pose_vector_iterator->start_position[gripp] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
					prev_pose_list_ptr();							//pozycje startowa nowego ruchu
				}
				if (gripp < (MAX_SERVOS_NR -1) ) {
					pose_vector_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_motor[i] == 0 || a_max_motor[i] == 0 || pose_vector_iterator->v[i] == 0 || pose_vector_iterator->a[i] == 0) {
						sr_ecp_msg.message("One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
					}

					pose_vector_iterator->v_r[i] = v_max_motor[i] * pose_vector_iterator->v[i];
					pose_vector_iterator->a_r[i] = a_max_motor[i] * pose_vector_iterator->a[i];

					pose_vector_iterator->v_p[i] = 0; //zalozenie, ze poczatkowa predkosc jest rowna 0

					if (type == lib::ABSOLUTE) {
						if(pose_vector_iterator->coordinates[i]-pose_vector_iterator->start_position[i] < 0) { //zapisanie kierunku dla pierwszego ruchu
							pose_vector_iterator->k[i] = -1;			//zapisanie kierunku nastepnych ruchow dokonywane jest dalej
						}
						else {
							pose_vector_iterator->k[i] = 1;
						}
					} else if (type == lib::RELATIVE) {
						if (pose_vector_iterator->coordinates[i] < 0) {
							pose_vector_iterator->k[i] = -1;
						} else {
							pose_vector_iterator->k[i] = 1;
						}
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

			case lib::ECP_XYZ_EULER_ZYZ:
				gripp = 6;
				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_vector_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_vector_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr();					//predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

		    	pose_vector_iterator->pos_num = j;
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_vector_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_vector_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_zyz[i] == 0 || a_max_zyz[i] == 0 || pose_vector_iterator->v[i] == 0 || pose_vector_iterator->a[i] == 0) {
						sr_ecp_msg.message("One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
					}
					pose_vector_iterator->v_r[i] = v_max_zyz[i] * pose_vector_iterator->v[i];
					pose_vector_iterator->a_r[i] = a_max_zyz[i] * pose_vector_iterator->a[i];
				}

				break;

			case lib::ECP_XYZ_ANGLE_AXIS:
				gripp = 6;
				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_vector_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_vector_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr();					//predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

		    	pose_vector_iterator->pos_num = j;
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_list_backup_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_vector_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

//				if (type == lib::RELATIVE) {//w obecnej implementacji zawsze spelniony
//					//nowa czesc			//TODO tego nie nalezy robic dla zwyklego relative tylko dla absolute zmienionego na relative
//					homog_matrix.set_from_xyz_angle_axis(pose_list_iterator->start_position);
//					homog_matrix2.set_from_xyz_angle_axis(pose_list_iterator->coordinates);
//					((!homog_matrix) * homog_matrix2).get_xyz_angle_axis(tmp_angle_axis_vector);
//					tmp_angle_axis_vector.to_table(pose_list_iterator->coordinates);
//
//					for (int n = 0; n < gripp; n++) {\
//						printf("pozycja %f\n", pose_list_iterator->coordinates[n]);
//					}
//				}

				pose_vector_iterator->coordinates[gripp] = pose_vector_iterator->coordinates[gripp] - pose_vector_iterator->start_position[gripp]; //nowa czesc

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_aa[i] == 0 || a_max_aa[i] == 0 || pose_vector_iterator->v[i] == 0 || pose_vector_iterator->a[i] == 0) {
						sr_ecp_msg.message("One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
					}
					pose_vector_iterator->v_r[i] = v_max_aa[i] * pose_vector_iterator->v[i];
					pose_vector_iterator->a_r[i] = a_max_aa[i] * pose_vector_iterator->a[i];
				}

				break;

			case lib::ECP_JOINT:

				if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
					gripp=7;
				} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
					gripp=6;
				}

				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_vector_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_vector_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr();					//predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

		    	pose_vector_iterator->pos_num = j;
				for (i = 0; i <= gripp; i++) {
					temp = pose_vector_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_vector_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

				if (gripp < (MAX_SERVOS_NR -1)) {//jesli postument
					pose_vector_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_joint[i] == 0 || a_max_joint[i] == 0 || pose_vector_iterator->v[i] == 0 || pose_vector_iterator->a[i] == 0) {
						sr_ecp_msg.message("One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
					}
					pose_vector_iterator->v_r[i] = v_max_joint[i] * pose_vector_iterator->v[i];
					pose_vector_iterator->a_r[i] = a_max_joint[i] * pose_vector_iterator->a[i];
				}

				break;

			case lib::ECP_MOTOR:

				if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
					gripp=7;
				} else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
					gripp=6;
				}

				//zapisanie v_p, musi byc tutaj bo wczesniej nie ma v_k poprzedniego ruchu
				for (i = 0; i < MAX_SERVOS_NR; i++) {
					temp = pose_vector_iterator->v_k[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_vector_iterator->v_p[i] = temp; //koncowa predkosc poprzedniego ruchu jest poczatkowa
						prev_pose_list_ptr();					//predkoscia nowego ruchu
					}
				}

				next_pose_list_ptr(); //GLOWNA INKREMENTACJA iteratora listy pose_list (bez powrotu)

		    	pose_vector_iterator->pos_num = j;
				for (i = 0; i <= gripp; i++) {
					temp = pose_vector_iterator->coordinates[i];
					if (!is_last_list_element()) {
						next_pose_list_ptr();
						pose_vector_iterator->start_position[i] = temp;//przypisanie pozycji koncowej poprzedniego ruchu jako
						prev_pose_list_ptr();							//pozycje startowa nowego ruchu
					}
				}

				if (gripp < (MAX_SERVOS_NR -1)) {//jesli postument
					pose_vector_iterator->start_position[gripp + 1] = 0.0;//TODO sprawdzic
				}

				for (i = 0; i < MAX_SERVOS_NR; i++) {
					if (v_max_motor[i] == 0 || a_max_motor[i] == 0 || pose_vector_iterator->v[i] == 0 || pose_vector_iterator->a[i] == 0) {
						sr_ecp_msg.message("One or more of 'v' or 'a' values is 0");
						throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
					}
					pose_vector_iterator->v_r[i] = v_max_motor[i] * pose_vector_iterator->v[i];
					pose_vector_iterator->a_r[i] = a_max_motor[i] * pose_vector_iterator->a[i];
				}

				break;

			default:
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
		} //end else (first interval)

		for(i=0;i<MAX_SERVOS_NR;i++) {//zapisanie coordinate_delta
			if (type == lib::ABSOLUTE) {
				td.coordinate_delta[i] = pose_vector_iterator->coordinates[i]-pose_vector_iterator->start_position[i];
			} else if (type == lib::RELATIVE) {
				td.coordinate_delta[i] = pose_vector_iterator->coordinates[i];
			}
		}
		if (!is_last_list_element()) {
			next_pose_list_ptr();
			if (type == lib::RELATIVE && td.arm_type == lib::ECP_XYZ_ANGLE_AXIS && external_absolute == true) {//w obecnej implementacji zawsze spelniony
				//nowa czesc			//TODO tego nie nalezy robic dla zwyklego relative tylko dla absolute zmienionego na relative

				for (int f = 0; f < gripp; f++) {\
					printf("pozycja startowa przed przeliczeniem %f\n", pose_vector_iterator->start_position[f]);
				}
				for (int g = 0; g < gripp; g++) {\
					printf("pozycja przed przeliczeniem %f\n", pose_vector_iterator->coordinates[g]);
				}

				homog_matrix.set_from_xyz_angle_axis(pose_vector_iterator->start_position);
				homog_matrix2.set_from_xyz_angle_axis(pose_vector_iterator->coordinates);
				((!homog_matrix) * homog_matrix2).get_xyz_angle_axis(tmp_angle_axis_vector);
				tmp_angle_axis_vector.to_table(pose_vector_iterator->coordinates);
				printf("poprzeliczało na wektor relatywny\n");
				for (int n = 0; n < gripp; n++) {\
					printf("pozycja %f\n", pose_vector_iterator->coordinates[n]);
				}
			}
			prev_pose_list_ptr();
		}

		for (i = 0; i < MAX_SERVOS_NR; i++) {//zapisanie v_r_next dla aktualnego ruchu i k dla nastepnego ruchu
			if (is_last_list_element()) {
				v_r_next[i] = 0.0;
			} else {
				temp = pose_vector_iterator->k[i];
				next_pose_list_ptr();

				if (type == lib::ABSOLUTE) {
					if (eq(pose_vector_iterator->coordinates[i],pose_vector_iterator->start_position[i])) {
						pose_vector_iterator->k[i] = -temp;//jesli droga jest rowna 0 w nastepnym ruchu to kierunek ustawiany jest na przeciwny aby robot zwolnil do 0
					} else if(pose_vector_iterator->coordinates[i]-pose_vector_iterator->start_position[i] < 0) {//nadpisanie k dla nastepnego ruchu
						pose_vector_iterator->k[i] = -1;
					} else {
						pose_vector_iterator->k[i] = 1;
					}
				} else if (type == lib::RELATIVE) {

//					if (external_absolute) {
//						printf("wchodzi w external\n");
//						if (eq(pose_list_iterator->coordinates[i],pose_list_iterator->start_position[i])) {
//							pose_list_iterator->k[i] = -temp;//jesli droga jest rowna 0 w nastepnym ruchu to kierunek ustawiany jest na przeciwny aby robot zwolnil do 0
//						} else if(pose_list_iterator->coordinates[i]-pose_list_iterator->start_position[i] < 0) {//nadpisanie k dla nastepnego ruchu
//							printf("ujemny\n");
//							pose_list_iterator->k[i] = -1;
//						} else {
//							pose_list_iterator->k[i] = 1;
//						}
//					} else {
						if (eq(pose_vector_iterator->coordinates[i],0)) {
							pose_vector_iterator->k[i] = -temp;//jesli droga jest rowna 0 w nastepnym ruchu to kierunek ustawiany jest na przeciwny aby robot zwolnil do 0
						} else if(pose_vector_iterator->coordinates[i] < 0) {//nadpisanie k dla nastepnego ruchu
							printf("ustawia ujemne k\n");
							pose_vector_iterator->k[i] = -1;
						} else {
							pose_vector_iterator->k[i] = 1;
						}
//					}
				}

				switch (td.arm_type) {

					case lib::ECP_XYZ_EULER_ZYZ:
						pose_vector_iterator->v_r[i] = v_max_zyz[i] * pose_vector_iterator->v[i];
						pose_vector_iterator->a_r[i] = a_max_zyz[i] * pose_vector_iterator->a[i];

						break;

					case lib::ECP_XYZ_ANGLE_AXIS:
						pose_vector_iterator->v_r[i] = v_max_aa[i] * pose_vector_iterator->v[i];
						pose_vector_iterator->a_r[i] = a_max_aa[i] * pose_vector_iterator->a[i];

						break;

					case lib::ECP_MOTOR:
						pose_vector_iterator->v_r[i] = v_max_motor[i] * pose_vector_iterator->v[i];
						pose_vector_iterator->a_r[i] = a_max_motor[i] * pose_vector_iterator->a[i];

						break;

					case lib::ECP_JOINT:
						pose_vector_iterator->v_r[i] = v_max_joint[i] * pose_vector_iterator->v[i];
						pose_vector_iterator->a_r[i] = a_max_joint[i] * pose_vector_iterator->a[i];

						break;
					default:
						throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
				}

				if (pose_vector_iterator->k[i] != temp) {
					v_r_next[i] = 0;
				} else {
					v_r_next[i] = pose_vector_iterator->v_r[i];
				}

				prev_pose_list_ptr(); // powrot do aktualnej pozycji
			}
		}
		// ==================================== poczatek oliczen ========================================

		t_max = 0;

		//obliczenie drogi dla kazdej osi
		for (i = 0; i < MAX_SERVOS_NR; i++) {
			if (type == lib::ABSOLUTE) {
				s[i]=fabs(pose_vector_iterator->coordinates[i] - pose_vector_iterator->start_position[i]);
			} else if (type == lib::RELATIVE) {
				s[i] = fabs(pose_vector_iterator->coordinates[i]);//tryb relative
			}
		}
		//sprawdzanie czy droga w etapach przyspieszenia i hamowania nie jest wieksza niz droga calego ruchu

		//warunki na modele ruchu dla wszystkich osi
		for (i = 0; i < MAX_SERVOS_NR; i++) { //petla w ktorej obliczany jest czas dla kazdej osi i sprawdzane jest czy da sie wykonac ruch w zalozonych etapach
			printf("=============================================\n");
			printf("v_p: %f\t v_r: %f\t v_r_next: %f\t a_r: %f\t v_k: %f\n", pose_vector_iterator->v_p[i], pose_vector_iterator->v_r[i], v_r_next[i], pose_vector_iterator->a_r[i], pose_vector_iterator->v_k[i]);
			flushall();

			if (s[i] < distance_eps) {//najmniejsza wykrywalna droga - obecnie 0
				//printf("droga 0 %d\n", i);
				pose_vector_iterator->model[i] = 0; //kinematic_model_with_tool 0... brak ruchu
				t[i] = 0;
				pose_vector_iterator->s_przysp[i] = 0;
				pose_vector_iterator->s_jedn[i] = 0;
				pose_vector_iterator->v_k[i] = 0;
				continue;
			}

			if (pose_vector_iterator->v_p[i] < pose_vector_iterator->v_r[i] && v_r_next[i] < pose_vector_iterator->v_r[i]) { //pierwszy kinematic_model_with_tool
				//printf("pierwszy kinematic_model_with_tool w osi %d\n", i);

				pose_vector_iterator->model[i] = 1;

				s_temp1[i] = pose_vector_iterator->v_p[i] * (pose_vector_iterator->v_r[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i] + (0.5 * pose_vector_iterator->a_r[i] * ((pose_vector_iterator->v_r[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i]) * ((pose_vector_iterator->v_r[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i]));
				s_temp2[i] = pose_vector_iterator->v_r[i] * (pose_vector_iterator->v_r[i] - v_r_next[i])/pose_vector_iterator->a_r[i] - (0.5 * pose_vector_iterator->a_r[i] * ((pose_vector_iterator->v_r[i] - v_r_next[i])/pose_vector_iterator->a_r[i]) * ((pose_vector_iterator->v_r[i] - v_r_next[i])/pose_vector_iterator->a_r[i]));

				t_temp1 = (pose_vector_iterator->v_r[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i];
				t_temp2 = (pose_vector_iterator->v_r[i] - v_r_next[i])/pose_vector_iterator->a_r[i];

				pose_vector_iterator->v_k[i] = v_r_next[i];

				//printf("s_temp1: %f\ts_temp2: %f\n", s_temp1[i], s_temp2[i]);
				//printf("t_temp1: %f\tt_temp2: %f\n", t_temp1, t_temp2);

				if (s_temp1[i] + s_temp2[i] > s[i]) {
					printf("redukcja predkosci w osi %d\n", i);
					//TODO tutaj wstawic optymalizacje czasu
					optimize_time1(pose_vector_iterator, i, s[i]);//nastepuje zapisanie czasu, mozliwe wywolanie vp_reduction lub vk_reduction wewnatrz
					if (trajectory_calculated == true) {
						return;
					}
					t[i] = pose_vector_iterator->t;
					//printf("t[i]: %f\n", t[i]);
					//t[i] = t_temp1 + t_temp2;
					pose_vector_iterator->t = t[i];
					reduction_model_1(pose_vector_iterator, i, s[i]);
					//printf("1 v_r: %f\n", pose_list_iterator->v_r[i]);
					if (trajectory_calculated == true) {
						return;
					}

				} else {//droga przyspieszenia i opoznienia nie przekracza drogi ruchu

					t[i] = t_temp1 + t_temp2 + (s[i] - (s_temp1[i] + s_temp2[i]))/pose_vector_iterator->v_r[i];
					pose_vector_iterator->s_przysp[i] = s_temp1[i];
					pose_vector_iterator->s_jedn[i] = s[i] - s_temp2[i] - s_temp1[i];
				}

            } else if (pose_vector_iterator->v_p[i] < pose_vector_iterator->v_r[i] && (v_r_next[i] > pose_vector_iterator->v_r[i] || eq(v_r_next[i], pose_vector_iterator->v_r[i]))) { // drugi kinematic_model_with_tool
            	//printf("drugi kinematic_model_with_tool w osi %d\n", i);

            	pose_vector_iterator->model[i] = 2;
            	s_temp1[i] = pose_vector_iterator->v_p[i] * (pose_vector_iterator->v_r[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i] + (0.5 * pose_vector_iterator->a_r[i] * ((pose_vector_iterator->v_r[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i]) * ((pose_vector_iterator->v_r[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i]));
        		t_temp1 = (pose_vector_iterator->v_r[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i];

            	pose_vector_iterator->v_k[i] = pose_vector_iterator->v_r[i];

				//printf("s_temp1: %f\n", s_temp1[i]);
				//printf("t_temp1: %f\n", t_temp1);

            	if (s_temp1[i] > s[i]) {
            		optimize_time2(pose_vector_iterator, i, s[i]);
            		t[i] = pose_vector_iterator->t;
            		//t[i] = t_temp1;
					vk_reduction(pose_vector_iterator, i, s[i], t[i]);

					if (trajectory_calculated == true) {
						return;
					}

            	} else {

            		t[i] = t_temp1 + (s[i] - s_temp1[i])/pose_vector_iterator->v_r[i];
    				pose_vector_iterator->s_przysp[i] = s_temp1[i];
    				pose_vector_iterator->s_jedn[i] = s[i] - s_temp1[i];
            	}

			} else if (eq(pose_vector_iterator->v_p[i], pose_vector_iterator->v_r[i]) && (v_r_next[i] > pose_vector_iterator->v_r[i] || eq(v_r_next[i], pose_vector_iterator->v_r[i]))) { //trzeci kinematic_model_with_tool
				//printf("trzeci kinematic_model_with_tool w osi %d\n", i);

				pose_vector_iterator->model[i] = 3;

				pose_vector_iterator->s_przysp[i] = 0;

				if (v_r_next[i] > pose_vector_iterator->v_r[i] || (eq(v_r_next[i],0) && eq(pose_vector_iterator->v_r[i],0))) { //ten przypadek wystepuje w rekurencji, po redukcji (prawdopodobnie po ostatnich zmianach juz nie wystepuje...)

					t[i] = pose_vector_iterator->t;
					if (v_r_next[i] != 0) {
						pose_vector_iterator->v_k[i] = v_r_next[i];
					}

					if (eq(pose_vector_iterator->v_r[i],0)) {
						pose_vector_iterator->s_jedn[i] = 0;
					} else {
						t_temp1 = (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_r[i])/pose_vector_iterator->a_r[i];
						pose_vector_iterator->s_jedn[i] = s[i] -
										((pose_vector_iterator->a_r[i] * t_temp1 * t_temp1)/2 + (pose_vector_iterator->v_r[i] * t_temp1));
					}

				} else {
					t[i] = s[i]/pose_vector_iterator->v_r[i];

					//printf("t: %f\n", t[i]);
					pose_vector_iterator->s_jedn[i] = s[i];
					pose_vector_iterator->v_k[i] = pose_vector_iterator->v_r[i];
				}
			} else if (eq(pose_vector_iterator->v_p[i], pose_vector_iterator->v_r[i]) && v_r_next[i] < pose_vector_iterator->v_r[i]) { //czwarty kinematic_model_with_tool
				//printf("czwarty kinematic_model_with_tool w osi %d\n", i);

				pose_vector_iterator->model[i] = 4;

				s_temp1[i] = pose_vector_iterator->v_r[i] * (pose_vector_iterator->v_r[i] - v_r_next[i])/pose_vector_iterator->a_r[i] - (0.5 * pose_vector_iterator->a_r[i] * ((pose_vector_iterator->v_r[i] - v_r_next[i])/pose_vector_iterator->a_r[i]) * ((pose_vector_iterator->v_r[i] - v_r_next[i])/pose_vector_iterator->a_r[i]));
				t_temp1 = (pose_vector_iterator->v_r[i] - v_r_next[i])/pose_vector_iterator->a_r[i];

				pose_vector_iterator->v_k[i] = v_r_next[i];

				//printf("s_temp1: %f\n", s_temp1[i]);
				//printf("t_temp1: %f\n", t_temp1);

				if (s_temp1[i] > s[i]) {
					optimize_time4(pose_vector_iterator, i, s[i]);
					t[i] = pose_vector_iterator->t;
					//t[i] = t_temp1;
					/*vp_reduction(pose_list_iterator, i, s[i], t[i]);
					if (trajectory_calculated == true) {
						return;
					}*/
/*					pose_vector_iterator->s_przysp[i] = 0;
					pose_vector_iterator->s_jedn[i] = 0;

				} else {

					t[i] = t_temp1 + (s[i] - s_temp1[i])/pose_vector_iterator->v_r[i];
					pose_vector_iterator->s_przysp[i] = 0;
					pose_vector_iterator->s_jedn[i] = s[i] - s_temp1[i];
				}

			} else {
				//printf("\n ten przypadek nigdy nie moze wystapic\n");
				//printf(" ********************** Error w osi %d *************************\n", i);
				//flushall();pose_list_iterator->t = t_max;
				sr_ecp_msg.message("Unexpected calculation error 1. Save your trajectory and report bug");
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
	    if(ceil(t_max/tk)*tk != t_max)//zaokraglenie czasu do wielokrotnosci trwania makrokroku
	    {
	        t_max = ceil(t_max/tk);
	        t_max = t_max*tk;
	        //printf("t_max = %f\n", t_max);
	        pose_vector_iterator->t = t_max;
	    }

		pose_vector_iterator->interpolation_node_no = lround(t_max / tk);
		//printf("liczba makrokrokow w ruchu %d\n", pose_list_iterator->interpolation_node_no);

		for (i = 0; i < MAX_SERVOS_NR; i++) {//obliczanie przysp i jedn a takze ewentualna redukcja predkosci z powodu zbyt krotkiego czasu

			//printf("czas ruchu w osi %d = %f\n", i, t[i]);

			//if (s[i] < distance_eps || t[i] == 0) {//jesli droga jest mniejsza od najmniejszej wykrywalnej albo czas jest rowny 0
			if (t[i] == 0) {
				//printf("droga 0 (koncowe obliczenia) w osi %d\n", i);
				pose_vector_iterator->przysp[i] = 0;
				pose_vector_iterator->jedn[i] = 0;
				continue;
			}

			if (eq(pose_vector_iterator->v_p[i], 0) && eq(pose_vector_iterator->v_r[i], 0)) {
				t[i] = t_max; //unikniecie dalszej redukcji w sytuacji gdy v_p i v_r wynosza 0 (mozna wtedy dowolnie wydluzycz czas postoju bez redukcji predkosci)
			}

			if (fabs(t[i] - t_max) > 0) {//redukcja predkosci w danej osi ze wzgledu na zbyt krotki czas ruchu
				if (pose_vector_iterator->model[i] == 1) { //kinematic_model_with_tool 1
					reduction_model_1(pose_vector_iterator, i, s[i]);
					//printf("2 v_r: %f\n", pose_list_iterator->v_r[i]);
					if (trajectory_calculated == true) {//wyjscie z rekurencji
						return;
					}
				} else if (pose_vector_iterator->model[i] == 2) {//kinematic_model_with_tool 2
					reduction_model_2(pose_vector_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else if (pose_vector_iterator->model[i] == 3) {//kinematic_model_with_tool 3
					reduction_model_3(pose_vector_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else if (pose_vector_iterator->model[i] == 4) {//kinematic_model_with_tool 4
					reduction_model_4(pose_vector_iterator, i, s[i]);
					if (trajectory_calculated == true) {
						return;
					}
				} else {
					//printf(" ten przypadek nie moze wystapic (redukcja ze wzgledu na czas)\n");
					sr_ecp_msg.message("Unexpected calculation error 2. Save your trajectory and report bug");
					throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
				}

				//printf("redukcja predkosci z powodu czasu w osi %d\n", i);
			}
			printf("v_p: %f\t v_r: %f\t v_r_next: %f\t a_r: %f\t v_k: %f\n", pose_vector_iterator->v_p[i], pose_vector_iterator->v_r[i], v_r_next[i], pose_vector_iterator->a_r[i], pose_vector_iterator->v_k[i]);
			pose_vector_iterator->przysp[i]=fabs((pose_vector_iterator->v_r[i]-pose_vector_iterator->v_p[i])/(pose_vector_iterator->a_r[i]*tk));//zapisanie makrokroku w ktorym konczy sie przyspieszanie
			pose_vector_iterator->jedn[i]=(t_max-(fabs(pose_vector_iterator->v_r[i]-pose_vector_iterator->v_k[i])/pose_vector_iterator->a_r[i]))/tk - pose_vector_iterator->przysp[i];//zapisanie makrokroku w ktorym konczy sie jednostajny
		    printf("przysp: %f\t jedn: %f\n", pose_vector_iterator->przysp[i], pose_vector_iterator->jedn[i]);
			//printf("jedn: %f\t t max: %f\t v_r: %f\t v_k: %f\t a_r: %f\t tk: %f\n",pose_list_iterator->jedn[i], t_max, pose_list_iterator->v_r[i], pose_list_iterator->v_k[i], pose_list_iterator->a_r[i], tk);
		}

		//obliczanie v_grip
		pose_vector_iterator->v_grip = (s[gripp]/pose_vector_iterator->t);

		switch (td.arm_type) {

			case lib::ECP_XYZ_EULER_ZYZ:
				if(pose_vector_iterator->v_grip < v_grip_min_zyz) {
					pose_vector_iterator->v_grip = v_grip_min_zyz;
				}
				break;

			case lib::ECP_XYZ_ANGLE_AXIS:
				if(pose_vector_iterator->v_grip < v_grip_min_aa) {
					pose_vector_iterator->v_grip = v_grip_min_aa;
					printf("ustawienie v_gripp na minimum\n");
				}
				break;

			case lib::ECP_MOTOR:
				if(pose_vector_iterator->v_grip < v_grip_min_motor) {
					pose_vector_iterator->v_grip = v_grip_min_motor;
				}
				break;

			case lib::ECP_JOINT:
				if(pose_vector_iterator->v_grip < v_grip_min_joint) {
					pose_vector_iterator->v_grip = v_grip_min_joint;
				}
				break;
			default:
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}

		//if (debug) {
			for (int os = 0; os < MAX_SERVOS_NR; os++) {
				printf("\n=============== pozycja trajektorii nr %d pos: %d ============== os: %d ====\n", j, pose_vector_iterator->pos_num, os);
				printf("czas ruchu %f\t model: %d\n", pose_vector_iterator->t, pose_vector_iterator->model[i]);
				printf("coordinates: %f\n", pose_vector_iterator->coordinates[os]);
				printf("jedn: %f\t przysp: %f\n", pose_vector_iterator->jedn[os], pose_vector_iterator->przysp[os]);
				printf("liczba makrokrokow: %d\n", pose_vector_iterator->interpolation_node_no);
				printf("start pos: %f\t kierunek (k): %f\n", pose_vector_iterator->start_position[os], pose_vector_iterator->k[os]);
				printf("s: %f\ns_przysp: %f\t s_jedn: %f\n", s[os], pose_vector_iterator->s_przysp[os], pose_vector_iterator->s_jedn[os]);
				printf("czas\t\tv_r\t\tv_r_next\ta_r\t\tv_p\t\tv_k\n");
				printf("%f\t%f\t%f\t%f\t%f\t%f\n", t[os], pose_vector_iterator->v_r[os], v_r_next[os], pose_vector_iterator->a_r[os], pose_vector_iterator->v_p[os], pose_vector_iterator->v_k[os]);
				printf("v_grip: %f\n\n", pose_vector_iterator->v_grip);
				flushall();
			}

			printf("************************************************** nowa pozycja **************************************************\n");
			printf("type: %d\n", type);
			flushall();
		//}
    }
    trajectory_calculated = true;

}*///end - calculate

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
