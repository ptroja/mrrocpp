/**
 * @file
 * @brief Contains definitions of the methods of constant_velocity class.
 * @author rtulwin
 * @ingroup generators
 */

#include "base/ecp/ecp_exceptions.h"
#include "ecp_g_constant_velocity.h"
#include "base/lib/datastr.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

using namespace std;

constant_velocity::constant_velocity(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
		multiple_position<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose,
		ecp::common::generator::trajectory_interpolator::constant_velocity_interpolator,
		ecp::common::generator::velocity_profile_calculator::constant_velocity_profile> (_ecp_task) {
	this->pose_spec = pose_spec;
	this->axes_num = axes_num;
	this->vpc = velocity_profile_calculator::constant_velocity_profile();
	this->inter = trajectory_interpolator::constant_velocity_interpolator();

	create_velocity_vectors(axes_num);
}

constant_velocity::~constant_velocity() {

}

void constant_velocity::print_pose_vector() {
	printf("------------------ Pose List ------------------\n");
	pose_vector_iterator = pose_vector.begin();
	int z;
	for (int k = 0; k < pose_vector.size(); k++) {
                printf("coord:\t");
		for (z = 0; z < pose_vector_iterator->coordinates.size(); z++) {
			printf("%f\t", pose_vector_iterator->coordinates[z]);
		}
		printf("\n");
		printf("s:\t");
		for (z = 0; z < pose_vector_iterator->s.size(); z++) {
			printf("%f\t", pose_vector_iterator->s[z]);
		}
		printf("\n");
		printf("k:\t");
		for (z = 0; z < pose_vector_iterator->k.size(); z++) {
			printf("%f\t", pose_vector_iterator->k[z]);
		}
		printf("\n");
		printf("times:\t");
		for (z = 0; z < pose_vector_iterator->s.size(); z++) {
			printf("%f\t", pose_vector_iterator->times[z]);
		}
		printf("\n");
		printf("v_r:\t");
		for (z = 0; z < pose_vector_iterator->v_r.size(); z++) {
			printf("%f\t", pose_vector_iterator->v_r[z]);
		}
		printf("\n");
		printf("t: %f\t pos_num: %d\t number of macrosteps: %d\n", pose_vector_iterator->t, pose_vector_iterator->pos_num, pose_vector_iterator->interpolation_node_no);
		flushall();
		pose_vector_iterator++;
	}
}

bool constant_velocity::calculate() {

	sr_ecp_msg.message("Calculating...");

	int i;//loop counter

	pose_vector_iterator = pose_vector.begin();
	if (pose_spec == lib::ECP_XYZ_ANGLE_AXIS && motion_type == lib::ABSOLUTE) {

		set_relative();
		angle_axis_absolute_transformed_into_relative = true;

		for (i = 0; i < pose_vector.size(); i++) {
			if (!vpc.calculate_relative_angle_axis_vector(pose_vector_iterator)) {
				if (debug) {
					printf("calculate_relative_angle_axis_vector returned false\n");
				}
				return false;
			}
			pose_vector_iterator++;
		}
	}

	pose_vector_iterator = pose_vector.begin();

	for (i = 0; i < pose_vector.size(); i++) {//calculate distances, directions, times and velocities for each pose and axis

		if(motion_type == lib::ABSOLUTE) {//absolute type of motion
			if (!vpc.calculate_absolute_distance_direction_pose(pose_vector_iterator)) {
				return false;
			}
		} else if(motion_type == lib::RELATIVE) {//relative type of motion
			if (!vpc.calculate_relative_distance_direction_pose(pose_vector_iterator)) {
				return false;
			}
		} else {
			sr_ecp_msg.message("Wrong motion type");
			BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(ECP_ERRORS));//TODO change the second argument
		}

		if(!vpc.calculate_time_pose(pose_vector_iterator) ||//calculate times for each of the axes
		!vpc.calculate_pose_time(pose_vector_iterator, mc) ||//calculate the longest time from each of the axes and set it as the pose time (also extend the time to be the multiplcity of a single macrostep time)
		!vpc.calculate_constant_velocity_pose(pose_vector_iterator)) {//calculate velocities for all of the axes according to the longest needed time
			return false;
		}

		//calculate the number of the macrosteps for the pose
		pose_vector_iterator->interpolation_node_no = ceil(pose_vector_iterator->t / mc);

		if (debug) {
			printf("interpolation node no: %d\n", pose_vector_iterator->interpolation_node_no);
		}

		pose_vector_iterator++;
	}

	return true;
}

void constant_velocity::create_velocity_vectors(int axes_num) {
	joint_velocity = vector<double>(axes_num, 0.05);
	joint_max_velocity = vector<double>(axes_num, 1.5);
	motor_velocity = vector<double>(axes_num, 0.05);
	motor_max_velocity = vector<double>(axes_num, 200.0);
	euler_zyz_velocity= vector<double>(axes_num, 0.01);//TODO check if this is a reasonable value
	euler_zyz_max_velocity = vector<double>(axes_num, 5.0);
	angle_axis_velocity = vector<double>(axes_num, 0.01);//TODO check if this is a reasonable value
	angle_axis_max_velocity = vector<double>(axes_num, 5.0);
}

//--------------- METHODS USED TO LOAD POSES ----------------

bool constant_velocity::load_absolute_joint_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_JOINT, joint_velocity, joint_max_velocity);
}

bool constant_velocity::load_relative_joint_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_JOINT, joint_velocity, joint_max_velocity);
}

bool constant_velocity::load_absolute_motor_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_MOTOR, motor_velocity, motor_max_velocity);
}

bool constant_velocity::load_relative_motor_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_MOTOR, motor_velocity, motor_max_velocity);
}

bool constant_velocity::load_absolute_euler_zyz_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_XYZ_EULER_ZYZ, euler_zyz_velocity, euler_zyz_max_velocity);
}

bool constant_velocity::load_relative_euler_zyz_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_XYZ_EULER_ZYZ, euler_zyz_velocity, euler_zyz_max_velocity);
}

bool constant_velocity::load_absolute_angle_axis_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_XYZ_ANGLE_AXIS, angle_axis_velocity, angle_axis_max_velocity);
}

bool constant_velocity::load_relative_angle_axis_trajectory_pose(const vector<double> & coordinates) {

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_XYZ_ANGLE_AXIS, angle_axis_velocity, angle_axis_max_velocity);
}

bool constant_velocity::load_absolute_pose(ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose & trajectory_pose) {
	if (trajectory_pose.arm_type == lib::ECP_JOINT) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, joint_max_velocity);
	} else if (trajectory_pose.arm_type == lib::ECP_MOTOR) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, motor_max_velocity);
	} else if (trajectory_pose.arm_type == lib::ECP_XYZ_ANGLE_AXIS) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, angle_axis_max_velocity);
	} else if (trajectory_pose.arm_type == lib::ECP_XYZ_EULER_ZYZ) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, euler_zyz_max_velocity);
	} else {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_POSE_SPECIFICATION));
	}
	return true;
}

bool constant_velocity::load_relative_pose(ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose & trajectory_pose) {
	if (trajectory_pose.arm_type == lib::ECP_JOINT) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, joint_max_velocity);
	} else if (trajectory_pose.arm_type == lib::ECP_MOTOR) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, motor_max_velocity);
	} else if (trajectory_pose.arm_type == lib::ECP_XYZ_ANGLE_AXIS) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, angle_axis_max_velocity);
	} else if (trajectory_pose.arm_type == lib::ECP_XYZ_EULER_ZYZ) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, euler_zyz_max_velocity);
	} else {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_POSE_SPECIFICATION));
	}
	return true;
}

bool constant_velocity::load_trajectory_pose(const vector<double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const vector<double> & v, const vector<double> & v_max) {

	if (!pose_vector.empty() && this->pose_spec != pose_spec) { //check if previous positions were provided in joint representation

		sr_ecp_msg.message("Representation different than the previous one");
		return false;
	}

	if (!pose_vector.empty() && this->motion_type != motion_type) {

		sr_ecp_msg.message("Wrong motion type");
		return false;
	}

	this->motion_type = motion_type;
	this->pose_spec = pose_spec;

	ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose pose; //new trajectory pose
	pose = ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose(pose_spec, coordinates, v); //create new trajectory pose
	pose.v_max = v_max; //set the v_max vector

	for (int j = 0; j < axes_num; j++) { //calculate v_r velocities
		pose.v_r[j] = pose.v[j] * pose.v_max[j];
	}

	if (pose_vector.empty()) {
		pose.pos_num = 1;
	} else {
		pose.pos_num = pose_vector.back().pos_num + 1;
	}

	if (motion_type == lib::ABSOLUTE) {
		if (!pose_vector.empty()) {//set the start position of the added pose as the desired position of the previous pose
			pose.start_position = pose_vector.back().coordinates;
		}
	}

	pose_vector.push_back(pose); //put new trajectory pose into a pose vector

	sr_ecp_msg.message("Pose loaded");

	return true;
}

bool constant_velocity::load_trajectory_from_file(const char* file_name)
{

        sr_ecp_msg.message(file_name);

        char coordinate_type_desc[80]; //description of pose specification read from the file
        char motion_type_desc[80]; //description of motion type read from the file
        lib::ECP_POSE_SPECIFICATION ps; //pose specification read from the file
        lib::MOTION_TYPE mt; //type of the commanded motion (relative or absolute)
        int number_of_poses = 0; //number of poses to be read
        int i, j; //loop counters

        std::ifstream from_file(file_name); // open the file
        if (!from_file.good()) {
                //perror(file_name);
                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_EXISTENT_FILE));
                return false;
        }

        if (!(from_file >> coordinate_type_desc)) {
                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
                return false;
        }

        //removing spaces and tabs
        i = 0;
        j = 0;
        while (coordinate_type_desc[i] == ' ' || coordinate_type_desc[i] == '\t')
                i++;
        while (coordinate_type_desc[i] != ' ' && coordinate_type_desc[i] != '\t' && coordinate_type_desc[i] != '\n'
                        && coordinate_type_desc[i] != '\r' && coordinate_type_desc[j] != '\0') {
                coordinate_type_desc[j] = toupper(coordinate_type_desc[i]);
                i++;
                j++;
        }
        coordinate_type_desc[j] = '\0';

        if (!strcmp(coordinate_type_desc, "MOTOR")) {
                ps = lib::ECP_MOTOR;
        } else if (!strcmp(coordinate_type_desc, "JOINT")) {
                ps = lib::ECP_JOINT;
        } else if (!strcmp(coordinate_type_desc, "XYZ_EULER_ZYZ")) {
                ps = lib::ECP_XYZ_EULER_ZYZ;
        } else if (!strcmp(coordinate_type_desc, "XYZ_ANGLE_AXIS")) {
                ps = lib::ECP_XYZ_ANGLE_AXIS;
        } else {
                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_TRAJECTORY_FILE));
                return false;
        }

        if (!(from_file >> number_of_poses)) {
                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
                return false;
        }

        flushall();
        if (!(from_file >> motion_type_desc)) {
                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
                return false;
        }

        if (!strcmp(motion_type_desc, "ABSOLUTE")) {
                mt = lib::ABSOLUTE;
        } else if (!strcmp(motion_type_desc, "RELATIVE")) {
                mt = lib::RELATIVE;
        } else {
                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_TRAJECTORY_FILE));
                return false;
        }

        double tab[10];
        int pos = from_file.tellg();
        char line[80];
        int dlugosc;
        do {
                from_file.getline(line, 80);
                dlugosc = strlen(line);
        } while (dlugosc < 5);
        int num = lib::setValuesInArray(tab, line);
        this->set_axes_num(num);
        from_file.seekg(pos);

        std::vector <double> v(axes_num); //vector of read velocities
        std::vector <double> a(axes_num); //vector of read accelerations
        std::vector <double> coordinates(axes_num); //vector of read coordinates

        for (i = 0; i < number_of_poses; i++) {
                for (j = 0; j < axes_num; j++) {
                        if (!(from_file >> v[j])) { //protection before the non-numerical data
                                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
                                return false;
                        }
                }
                from_file.ignore(std::numeric_limits <std::streamsize>::max(), '\n');
                for (j = 0; j < axes_num; j++) {
                        if (!(from_file >> a[j])) { //protection before the non-numerical data
                                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
                                return false;
                        }
                }
                from_file.ignore(std::numeric_limits <std::streamsize>::max(), '\n');
                for (j = 0; j < axes_num; j++) {
                        if (!(from_file >> coordinates[j])) { //protection before the non-numerical data
                                BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
                                return false;
                        }
                }
                from_file.ignore(std::numeric_limits <std::streamsize>::max(), '\n');

                if (ps == lib::ECP_MOTOR) {
                        load_trajectory_pose(coordinates, mt, ps, v, motor_max_velocity);
                } else if (ps == lib::ECP_JOINT) {
                        load_trajectory_pose(coordinates, mt, ps, v, joint_max_velocity);
                } else if (ps == lib::ECP_XYZ_EULER_ZYZ) {
                        load_trajectory_pose(coordinates, mt, ps, v, euler_zyz_max_velocity);
                } else if (ps == lib::ECP_XYZ_ANGLE_AXIS) {
                        load_trajectory_pose(coordinates, mt, ps, v, angle_axis_max_velocity);
                }
        }

        return true;
}
//--------------- METHODS USED TO LOAD POSES END ----------------

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
