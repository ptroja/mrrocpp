#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <cstdlib>

#include "base/ecp/ecp_task.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_st_acq_force.h"
#include "ecp_st_acquisition.h"
#include "sensor/pcbird/ecp_mp_s_pcbird.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_matrix.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

//Constructors
acq_force::acq_force(task::task &_ecp_t) :
	acquisition(_ecp_t)
{
	if (sub_task::ecp_t.config.section_name == lib::irp6ot_m::ECP_SECTION) {
		sub_task::ecp_t.ecp_m_robot = new irp6ot_m::robot(_ecp_t);
		sub_task::sr_ecp_msg.message("IRp6ot loaded");
	} else if (sub_task::ecp_t.config.section_name == lib::irp6p_m::ECP_SECTION) {
		sub_task::ecp_t.ecp_m_robot = new irp6p_m::robot(_ecp_t);
		sub_task::sr_ecp_msg.message("IRp6p loaded");
	}

	sub_task::ecp_t.sensor_m[ecp_mp::sensor::SENSOR_PCBIRD]
			= new ecp_mp::sensor::pcbird("[vsp_pcbird]", *_ecp_t.sr_ecp_msg, _ecp_t.config);
	sub_task::ecp_t.sensor_m[ecp_mp::sensor::SENSOR_PCBIRD]->configure_sensor();

	bias_run = new common::generator::bias_edp_force(_ecp_t);

	nose_run = new common::generator::pcbird_nose_run(_ecp_t, 8);
	nose_run->configure_pulse_check(true);
	nose_run->sensor_m = sub_task::ecp_t.sensor_m;

	sub_task::sr_ecp_msg.message("ecp loaded kcz_force");
}
;

void acq_force::conditional_execution()
{

}

void acq_force::write_data(const std::string & _K_fp, const std::string & _kk_fp, const std::string & _M_fp, const std::string & _mm_fp, int _number_of_measures)
{
	K_fp = _K_fp;
	kk_fp = _kk_fp;
	M_fp = _M_fp;
	mm_fp = _mm_fp;
	number_of_measures = _number_of_measures;
	remove(K_fp.c_str());
	remove(kk_fp.c_str());
	remove(M_fp.c_str());
	remove(mm_fp.c_str());

	acq_force::main_task_algorithm();
}

void acq_force::main_task_algorithm(void)
{
	sub_task::sr_ecp_msg.message("ecp kcz_force ready");

	int i, j, t;
	FILE *FP;
	//	char buffer[60];
	gsl_matrix *M = gsl_matrix_alloc(3, 3);
	gsl_matrix *K = gsl_matrix_alloc(3, 3);
	gsl_vector *m = gsl_vector_alloc(3);
	gsl_vector *k = gsl_vector_alloc(3);

	bias_run->Move();

	for (i = 0; i < number_of_measures; i++) {
		//move the robot + get the data
		nose_run->Move();

		//write the data to files
		//robot (K,k)
		//(rotation matrix & meters)
		for (j = 0; j < 3; j++)
			for (t = 0; t < 3; t++)
				gsl_matrix_set(K, j, t, sub_task::ecp_t.ecp_m_robot->reply_package.arm.pf_def.arm_frame[j][t]);
		for (j = 0; j < 3; j++)
			gsl_vector_set(k, j, sub_task::ecp_t.ecp_m_robot->reply_package.arm.pf_def.arm_frame[j][3]);
		//pcbird (M,m)
		//(degrees & meters)
		//returns Euler angles:
		//a=alpha=azimuth=Zang, b=beta=elevation=Yang, g=gamma=roll=Xang
		/*
		 from the manual:
		 "Zang (Azimuth) takes on values between the binary equivalent of +/- 180 degrees.
		 Yang (Elevation) takes on values between +/- 90 degrees, and Xang (Roll) takes on
		 values between +/- 180 degrees. As Yang (Elevation) approaches +/- 90 degrees, the
		 Zang (Azimuth) and Xang (Roll) become very noisy and exhibit large errors. At 90
		 degrees the Zang (Azimuth) and Xang (Roll) become undefined. This behavior is not a
		 limitation of the pcBIRD -- it is an inherent characteristic of these Euler angles."
		 */
		//conversion from Euler angles ZYX to rotation matrix
		//conversion to radians first
		ecp_mp::sensor::pcbird * bird =
				dynamic_cast <ecp_mp::sensor::pcbird *> (sub_task::ecp_t.sensor_m[ecp_mp::sensor::SENSOR_PCBIRD]);
		float Zang = bird->image.a * M_PI / 180;
		float Yang = bird->image.b * M_PI / 180;
		float Xang = bird->image.g * M_PI / 180;
		//conversion to matrix (pcbirdmanual, p.94)
		gsl_matrix_set(M, 0, 0, cos(Yang) * cos(Zang));
		gsl_matrix_set(M, 0, 1, cos(Yang) * sin(Zang));
		gsl_matrix_set(M, 0, 2, -sin(Yang));
		gsl_matrix_set(M, 1, 0, -cos(Xang) * sin(Zang) + sin(Xang) * sin(Yang) * cos(Zang));
		gsl_matrix_set(M, 1, 1, cos(Xang) * cos(Zang) + sin(Xang) * sin(Yang) * sin(Zang));
		gsl_matrix_set(M, 1, 2, sin(Xang) * cos(Yang));
		gsl_matrix_set(M, 2, 0, sin(Xang) * sin(Zang) + cos(Xang) * sin(Yang) * cos(Zang));
		gsl_matrix_set(M, 2, 1, -sin(Xang) * cos(Zang) + cos(Xang) * sin(Yang) * sin(Zang));
		gsl_matrix_set(M, 2, 2, cos(Xang) * cos(Yang));
		gsl_vector_set(m, 0, bird->image.x);
		gsl_vector_set(m, 1, bird->image.y);
		gsl_vector_set(m, 2, bird->image.z);

		FP = fopen(M_fp.c_str(), "a");
		gsl_matrix_fprintf(FP, M, "%g");
		fclose(FP);
		FP = fopen(mm_fp.c_str(), "a");
		gsl_vector_fprintf(FP, m, "%g");
		fclose(FP);
		FP = fopen(K_fp.c_str(), "a");
		gsl_matrix_fprintf(FP, K, "%g");
		fclose(FP);
		FP = fopen(kk_fp.c_str(), "a");
		gsl_vector_fprintf(FP, k, "%g");
		fclose(FP);

		/*
		 float tempx = bird->image.x;
		 float tempy = bird->image.y;
		 float tempz = bird->image.z;
		 float temp00 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[0][0];
		 float temp01 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[0][1];
		 float temp02 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[0][2];
		 float temp03 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[0][3];
		 float temp10 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[1][0];
		 float temp11 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[1][1];
		 float temp12 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[1][2];
		 float temp13 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[1][3];
		 float temp20 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[2][0];
		 float temp21 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[2][1];
		 float temp22 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[2][2];
		 float temp23 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[2][3];
		 sr_ecp_msg->message("Sensor:");
		 sprintf(buffer, "%f6.3 %f6.3 %f6.3", tempx, tempy, tempz);
		 sr_ecp_msg->message(buffer);
		 sr_ecp_msg->message("Robot:");
		 sprintf(buffer, "%f6.3 %f6.3 %f6.3 %f6.3", temp00, temp01, temp02, temp03);
		 sr_ecp_msg->message(buffer);
		 sprintf(buffer, "%f6.3 %f6.3 %f6.3 %f6.3", temp10, temp11, temp12, temp13);
		 sr_ecp_msg->message(buffer);
		 sprintf(buffer, "%f6.3 %f6.3 %f6.3 %f6.3", temp20, temp21, temp22, temp23);
		 sr_ecp_msg->message(buffer);
		 */
	}
}
;

} // namespace task
} // namespace common

namespace common {
namespace task {

//task* return_created_ecp_task(lib::configurator &_config){
//	return new kcz_force(_config);
//}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


