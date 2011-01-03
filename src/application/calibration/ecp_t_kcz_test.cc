#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <iostream>

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_t_kcz_test.h"
#include "sensor/pcbird/ecp_mp_s_pcbird.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
kcz_test::kcz_test(lib::configurator &_config): task(_config)
{
    if (config.section_name == lib::irp6ot_m::ECP_SECTION)
    {
        ecp_m_robot = new irp6ot_m::robot (*this);
    }
    else if (config.section_name == lib::irp6p_m::ECP_SECTION)
    {
        ecp_m_robot = new irp6p_m::robot (*this);
    }

	sensor_m[ecp_mp::sensor::SENSOR_PCBIRD] = new ecp_mp::sensor::pcbird("[vsp_pcbird]", *this->sr_ecp_msg, this->config);
	sensor_m[ecp_mp::sensor::SENSOR_PCBIRD]->configure_sensor();

	smoothgen2 = new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, 6);
	smoothgen2->sensor_m = sensor_m;

	sr_ecp_msg->message("ecp loaded kcz_test");
};

void kcz_test::main_task_algorithm(void ) {
	sr_ecp_msg->message("ecp kcz_test ready");

	std::vector <double> coordinates(6);

	smoothgen2->set_absolute();

	ecp_mp::sensor::pcbird * bird = dynamic_cast<ecp_mp::sensor::pcbird *> (sensor_m[ecp_mp::sensor::SENSOR_PCBIRD]);
	smoothgen2->reset();
	coordinates[0] = -0.150;
	coordinates[1] = 1.12;
	coordinates[2] = -0.1;
	coordinates[3] = 0.729*3.14;
	coordinates[4] = 0.685*3.14;
	coordinates[5] = -0.001*3.14;
	smoothgen2->load_absolute_angle_axis_trajectory_pose(coordinates);
	smoothgen2->Move();
	//smoothgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, -0.150, 1.12, -0.1, 0.729*3.14, 0.685*3.14, -0.001*3.14, 0.09, 0.000, true);


	smoothgen2->reset();
	coordinates[0] = bird->image.x - 0.421;
	coordinates[1] = -1.0 * bird->image.y + 1.119;
	coordinates[2] = -1.0 * bird->image.z - 0.22 + 0.20;
	coordinates[3] = 0.729*3.14;
	coordinates[4] = 0.685*3.14;
	coordinates[5] = -0.001*3.14;
	smoothgen2->load_absolute_angle_axis_trajectory_pose(coordinates);
	smoothgen2->Move();

	/*smoothgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS,
						bird->image.x - 0.421,
						-1.0 * bird->image.y + 1.119,
						-1.0 * bird->image.z - 0.22 + 0.20,
						0.729*3.14,
						0.685*3.14,
						-0.001*3.14,
						0.09, 0.000, true);*/
	smoothgen2->reset();

	smoothgen2->set_relative();

	//double vv[lib::MAX_SERVOS_NR]={0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
	//double aa[lib::MAX_SERVOS_NR]={0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

	float lastx = bird->image.x;
	float lasty = bird->image.y;
	float lastz = bird->image.z;

	float Zang = bird->image.a * M_PI / 180;
	float Yang = bird->image.b * M_PI / 180;
	float Xang = bird->image.g * M_PI / 180;
	double lastrot[3][3];
	lastrot[0][0] = cos(Yang)*cos(Zang);
	lastrot[0][1] = cos(Yang)*sin(Zang);
	lastrot[0][2] = -sin(Yang);
	lastrot[1][0] = -cos(Xang)*sin(Zang)+sin(Xang)*sin(Yang)*cos(Zang);
	lastrot[1][1] = cos(Xang)*cos(Zang)+sin(Xang)*sin(Yang)*sin(Zang);
	lastrot[1][2] = sin(Xang)*cos(Yang);
	lastrot[2][0] = sin(Xang)*sin(Zang)+cos(Xang)*sin(Yang)*cos(Zang);
	lastrot[2][1] = -sin(Xang)*cos(Zang)+cos(Xang)*sin(Yang)*sin(Zang);
	lastrot[2][2] = cos(Xang)*cos(Yang);
	lib::Homog_matrix matrix;
	lib::Xyz_Angle_Axis_vector temp1;
	matrix.set_rotation_matrix(lastrot);
	matrix.get_xyz_angle_axis(temp1);
	float lasta = temp1[3];
	float lastb = temp1[4];
	float lastc = temp1[5];

	float movex, movey, movez, movea, moveb, movec;
	float delta = 0.001;
	do {
    movex = abs(bird->image.y - lasty) > delta ? bird->image.y - lasty : 0;
    movey = abs(bird->image.x - lastx) > delta ? bird->image.x - lastx : 0;
    movez = abs(bird->image.z - lastz) > delta ? bird->image.z - lastz : 0;

	Zang = bird->image.a * M_PI / 180;
	Yang = bird->image.b * M_PI / 180;
	Xang = bird->image.g * M_PI / 180;
	lastrot[0][0] = cos(Yang)*cos(Zang);
	lastrot[0][1] = cos(Yang)*sin(Zang);
	lastrot[0][2] = -sin(Yang);
	lastrot[1][0] = -cos(Xang)*sin(Zang)+sin(Xang)*sin(Yang)*cos(Zang);
	lastrot[1][1] = cos(Xang)*cos(Zang)+sin(Xang)*sin(Yang)*sin(Zang);
	lastrot[1][2] = sin(Xang)*cos(Yang);
	lastrot[2][0] = sin(Xang)*sin(Zang)+cos(Xang)*sin(Yang)*cos(Zang);
	lastrot[2][1] = -sin(Xang)*cos(Zang)+cos(Xang)*sin(Yang)*sin(Zang);
	lastrot[2][2] = cos(Xang)*cos(Yang);
	lib::Homog_matrix matrix;
	matrix.set_rotation_matrix(lastrot);
	matrix.get_xyz_angle_axis(temp1);

    movea = abs(temp1[3] - lasta) > delta ? temp1[3] - lasta : 0;
    moveb = abs(temp1[4] - lastb) > delta ? temp1[4] - lastb : 0;
    movec = abs(temp1[5] - lastc) > delta ? temp1[5] - lastc : 0;

	coordinates[0] = -1.0 * movex;
	coordinates[1] = 1.0 * movey;
	coordinates[2] = 1.0 * movez;
	coordinates[3] = 1.0 * moveb;
	coordinates[4] = -1.0 * movea;
	coordinates[5] = -1.0 * movec;
	smoothgen2->load_relative_angle_axis_trajectory_pose(coordinates);

    /*smoothgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa,
															-1.0 * movex,
															1.0 * movey,
															1.0 * movez,
															1.0 * moveb,
															-1.0 * movea,
															-1.0 * movec,
															0.0, 0.0, true);*/
	lastx = bird->image.x;
	lasty = bird->image.y;
	lastz = bird->image.z;
	lasta = temp1[3];
	lastb = temp1[4];
	lastc = temp1[5];
	smoothgen2->Move();
	} while(true);

	smoothgen2->reset();

	ecp_termination_notice();
};

}
} // namespace common

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new common::task::kcz_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


