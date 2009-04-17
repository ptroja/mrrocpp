/*
 * ecp_g_spots_recognition.cc
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#include "ecp/irp6_on_track/ecp_g_spots_recognition.h"
#include <unistd.h>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

using namespace std;


spots::spots (common::task::task& _ecp_task)
        : generator (_ecp_task)
{
	//tool to plate frame initialization, all in rad and meters
	//spots in plate coordinates
	c = new CameraToTool();
	no_of_tcg_in_one = 0;
}

bool spots::first_step()
{
	sensor = (ecp_mp::sensor::cvfradia *)sensor_m[lib::SENSOR_CVFRADIA];

	//proste zadanie kinematyki
	the_robot->EDP_data.instruction_type = lib::GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = lib::FRAME;

	//zadanie zrobienia zdjec od fraidii
	sensor->to_vsp.command = 38;
	sensor->to_vsp.i_code = lib::VSP_INITIATE_READING;

	iter = 0;

	return true;
}

bool spots::next_step()
{
	if(iter == 0) //first time next_step
	{
	    get_frame(teg);
	    iter++;
	    return true;
	}

	// jesli zdjecia niegotowe, return true
    sensor->to_vsp.command = 0;

	//printf("%d\n", sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count);

    if(sensor->from_vsp.vsp_report == lib::VSP_REPLY_OK && sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count <= -2)
		//this position failed
    {
    	//beep();
    	std::cout<<"\a"<<std::endl;
    	std::cout<<"\a"<<std::endl;
		return false;
//<<<<<<< .mine
    }
//    else if(sensor->from_vsp.vsp_report != VSP_REPLY_OK || sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count <= 0)
//=======
    else if(sensor->from_vsp.vsp_report != lib::VSP_REPLY_OK || sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count <= 0)
//>>>>>>> .r2096
		return true;

		//teraz zabawa z wynikami
		get_pic();
		save_position();
		std::cout<<"\a"<<std::endl;

		return false;
}

void spots::get_frame(double t[12])
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<4; j++)
			t[4*i+j] = the_robot->EDP_data.current_arm_frame[i][j];
	}
}

void spots::get_pic()
{
	calib_data.sensor_union.sp_r.dz = sensor->from_vsp.comm_image.sensor_union.sp_r.dz;
	calib_data.sensor_union.sp_r.pic_count = sensor->from_vsp.comm_image.sensor_union.sp_r.pic_count;
	for (int i=0; i<4; i++)
	{
		calib_data.sensor_union.sp_r.x[i] = sensor->from_vsp.comm_image.sensor_union.sp_r.x[i];
		calib_data.sensor_union.sp_r.y[i] = sensor->from_vsp.comm_image.sensor_union.sp_r.y[i];
		calib_data.sensor_union.sp_r.z[i] = sensor->from_vsp.comm_image.sensor_union.sp_r.z[i];
	}
}

void spots::compute_TCE()
{
	double vec_cam1[3], vec_cam2[3], vec_cam3[3], vec_cam4[3];

	for (int i=0; i<4; i++)
	{
		switch(i)
		{
		  case 0:
			vec_cam1[0] = calib_data.sensor_union.sp_r.x[i];
			vec_cam1[1] = calib_data.sensor_union.sp_r.y[i];
			vec_cam1[2] = calib_data.sensor_union.sp_r.z[i];
			break;
		  case 1:
			vec_cam2[0] = calib_data.sensor_union.sp_r.x[i];
			vec_cam2[1] = calib_data.sensor_union.sp_r.y[i];
			vec_cam2[2] = calib_data.sensor_union.sp_r.z[i];
			break;
		  case 2:
			vec_cam3[0] = calib_data.sensor_union.sp_r.x[i];
			vec_cam3[1] = calib_data.sensor_union.sp_r.y[i];
			vec_cam3[2] = calib_data.sensor_union.sp_r.z[i];
			break;
		  case 3:
			vec_cam4[0] = calib_data.sensor_union.sp_r.x[i];
			vec_cam4[1] = calib_data.sensor_union.sp_r.y[i];
			vec_cam4[2] = calib_data.sensor_union.sp_r.z[i];
			break;
		}
	}

	double norm = c->computeTCE(vec_cam1, vec_cam2, vec_cam3, vec_cam4, tce);

	cout << "tce=[ " << endl;
	print_matrix(tce);
	cout << "];" << endl;

	cout << "vec1 = [" << vec_cam1[0] << "  " << vec_cam1[1] << "  " << vec_cam1[2] << " 1 ]'" << endl;
    cout << "vec2 = [" << vec_cam2[0] << "  " << vec_cam2[1] << "  " << vec_cam2[2] << " 1 ]'" << endl;
	cout << "vec3 = [" << vec_cam3[0] << "  " << vec_cam3[1] << "  " << vec_cam3[2] << " 1 ]'" << endl;
	cout << "vec4 = [" << vec_cam4[0] << "  " << vec_cam4[1] << "  " << vec_cam4[2] << " 1 ]'" << endl;
}

//<<<<<<< .mine
//void ecp_spots_generator::compute_TCG(double tcg_local[12])
//=======
void spots::compute_TCG(double tcg_local[12])
//>>>>>>> .r2096
{
	//T_C^G = T_E^G * T_C^E
	common::T_MatrixManip Teg_mm(teg);

	cout << "teg=[ " << endl;
	print_matrix(teg);
	cout << "];" << endl;

	Teg_mm.multiply_r_matrix4x4(tce, tcg_local);

	cout << "tcg=[ " << endl;
	print_matrix(tcg_local);
	cout << "];" << endl;
}

void spots::save_position()
{
	double tcg_local[12];

	compute_TCE();
	compute_TCG(tcg_local);

	for(int i=0; i<12; i++)
		tcg[i]+=(tcg_local[i]/16);
}

void spots::print_matrix(double m[12])
{
	cout << m[0] << "  " << m[1] << "  " << m[2] << "  " << m[3] << endl;
	cout << m[4] << "  " << m[5] << "  " << m[6] << "  " << m[7] << endl;
	cout << m[8] << "  " << m[9] << "  " << m[10] << "  " << m[11] << endl;
	cout << "0 0 0 1" << endl;
}

double spots::move_and_return(double pos_hist)
{
	double ret[12];
	double sum=0.;

	get_frame(ret);

	for (int i=3; i<12; i+=4)
		sum += (ret[i]*ret[i]);

	if(fabs(sum-pos_hist) < 0.2)
		return 0.;

	Move();
	return sum;

}

/*void ecp_spots_generator::save_position()
{
	Matrix4x4 T; // tego szukamy
	Matrix4x4 D;
	double d_matrix[16];
	double t_matrix[16];
	double * x;
	double c[4];

	//Matrix4x4 t_t_g(ground_to_tool, TYPE_T);
	//double * tool_to_ground = t_t_g.getinv();

	//Matrix4x4 p_t_t(tool_to_plate, TYPE_T);
	//double * plate_to_tool = p_t_t.getinv();

	Matrix4x4 p_t_g(tool_to_ground, TYPE_T);
	p_t_g.rproduct4x4(plate_to_tool);
	//double * plate_to_ground = p_t_g.getA();

FILE *fd = fopen("../msr/kalibracja.txt", "a+");
	for(int i=0; i<4; i++)
	{
		double vec_plate[4];
		double x, y;

		switch(i)
		{
		  case 0:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = vec_1[i];
			  break;
		  case 1:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = vec_2[i];
			  break;
		  case 2:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = vec_3[i];
			  break;
		  case 3:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = vec_4[i];
			  break;
		  default:
			  for(int i=0; i<4; i++)
				  vec_plate[i] = 0.;
			  break;
		}
		p_t_g.product4x1(vec_plate); // vec_plate := vec_ground

//FILE *fd1 = fopen("../msr/temp.txt", "a+");
//for(int it = 0; it<4; it++)
//		fprintf(fd1, "%f ", tool_to_ground[4*i+it]);
//for(int it=0; it<4; it++)
//	fprintf(fd1, "%f ",vec_plate[it]);
//fprintf(fd1, "\n");
//fclose(fd1);

for(int it=0; it<4; it++)
	fprintf(fd, "%f ",vec_plate[it]);
fprintf(fd, "%f %f %f 1.0 %f\n", 0.01*calib_data.sensor_union.sp_r.x[i], 0.01*calib_data.sensor_union.sp_r.y[i], 0.01*calib_data.sensor_union.sp_r.z[i], calib_data.sensor_union.sp_r.dz);
		//double vec_cam[3];
		//vec_cam[0] = calib_data.sensor_union.sp_r.x[i];
		//vec_cam[1] = calib_data.sensor_union.sp_r.y[i];
		//vec_cam[2] = calib_data.sensor_union.sp_r.z[i];

		for(int j=0; j<4; j++)
		    d_matrix[4*i+j] = vec_plate[j];
	}

	D.setA(d_matrix, TYPE_D);



//	for (int i=0; i<4; i++)
//	{
//		for(int j=0; j<4; j++)
//			fprintf(fd, "%f  ", d_matrix[4*i+j]);
//		fprintf(fd, "\n");
//	}fprintf(fd, "\n");

	//after that vec_plate is spot postion in ground coordinates
	//computation described in my thesis
	//D matrix's rows are spots' x,y,zs and 1s
	//c vectors are spots' xs then ys and finally zs
	//x vectors are rows of T

	for (int i=0; i<3; i++)
	{
		switch (i)
		{
		  case 0:
			for (int k=0; k<4; k++)
				c[k] = 0.01*calib_data.sensor_union.sp_r.x[k];
			break;
		  case 1:
			for (int k=0; k<4; k++)
				c[k] = 0.01*calib_data.sensor_union.sp_r.y[k];
			break;
		  case 2:
			for (int k=0; k<4; k++)
				c[k] = 0.01*calib_data.sensor_union.sp_r.z[k];
			break;
		  default:
			  break;
		}
//		D.setb(c);
//		D.solveAxb4x4();
//		x = D.getx();
//
//		for (int j=0; j<4; j++)
//			t_matrix[4*j+i] = x[j];
	}
//	T.setA(t_matrix, TYPE_T);

//	for (int i=0; i<4; i++)
//	{
//		for(int j=0; j<4; j++)
//			printf("%.3f  ", t_matrix[4*i+j]);
//		printf("\n");
//	}printf("\n\n");
fclose(fd);
}*/

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


