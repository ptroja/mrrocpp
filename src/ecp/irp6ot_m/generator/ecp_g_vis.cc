// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - force methods
// Funkcje do tworzenia procesow ECP z wykorzsytaniem sily
//
// Ostatnia modyfikacja: 2004r.
// -------------------------------------------------------------------------


#include <stdio.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6_on_track/generator/ecp_g_vis.h"

#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

#define PRINT 1


// CZY TE OCZY MOGA KLAMAC? GENERATOR SIODMEGO

float delta_prev=0;
float delta_last=0;
float v=0;
float delta_debug=0;
// float delta_x_prev=0;
// float delta_x_last=0;
// float delta_x_debug=0;
struct timespec crr_time, s_time, e_time;

long int nr=0;

int iter=0;

double xalfa;
double xbeta;
double xgamma;

double alfa;
double beta;
double gammax;

//double beta_old;

double xx;
double xy;
double xz;

int the_second=0;


double tablica[6];

double measure[6][5];
double measure_v[6][5];
double measure_a[6][5];
double measure_d[6][5];
double measure_d2[6][5];

double stearing[6][5];
double stearing_v[6][5];
double stearing_a[6][5];
double stearing_d[6][5];
double stearing_d2[6][5];

int debug=0;

double measure_border_u[]={1.000, 0.220, 0.338, 0.0, 1.57, 3.12}; //Zmienic ogranicz Z
double measure_border_d[]={0.810, -0.250, 0.130, -0.606, 1.32, 2.237}; //by na Z 0.078

//double d_u_max[6]={0.005, 0.1, 0.005, 0.005, 0.005, 0.005};
//double d_u_max[6]={0.005, 0.00625, 0.005, 0.005, 0.005, 0.005};
double d_u_max[6]={0.005, 0.0125, 0.005, 0.01, 0.005, 0.01};
double d2_u_max[6]={0.001, 0.001, 0.001, 0.01, 0.005, 0.005}; //tylko ustalone dla Y

int valid_measure;

double x2g=-0.1; //x nibytoola

int phase = 0;

int steps2switch=0;

//int back=0;

// extern FILE* fp1;

// double frame1[4][4];

	seven_eye_run_linear::seven_eye_run_linear(common::task::task& _ecp_task, int step):
		generator (_ecp_task) { 		step_no = step;          	};


bool seven_eye_run_linear::first_step (  ) {
  // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
  // Funkcja zwraca false gdy koniec generacji trajektorii
  // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana


// if ((mp_comm_counter++)==100) {// by Y - lekka manianka
// 	mp_comm_counter=0;

      for (int i=0; i<6; i++)
    delta[i]=0.0;

the_first=1;
the_second=0;

for (int i=0; i<6; i++)
  for (int j=0; j<5; j++)
measure[i][j]=0;


(sensor_m.begin())->second->base_period=0;
 (sensor_m.begin())->second->current_period=0;




 td.interpolation_node_no = 1;
		   td.internode_step_no = 40; //40
		   td.value_in_step_no = td.internode_step_no - 10; // 50


/*
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][0]=0.706825; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][0]=-0.706825;
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][0]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][0]=0;

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][1]=0.706825; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][1]=0.706825;
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][1]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][1]=0;

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][2]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][2]=0;
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][2]=1; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][2]=0;
*/
/*
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][0]=cos(M_PI/4); the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][0]=-sin(M_PI/4);
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][0]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][0]=0;

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][1]=sin(M_PI/4); the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][1]=cos(M_PI/4);
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][1]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][1]=0;

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][2]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][2]=0;
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][2]=1; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][2]=0;
*/




// 	A_B_T_cur.to_table();

xalfa=0;
xbeta=0; // -1*(M_PI-2.769);
xgamma=2.40;

xx=0;
xy=0;
xz=0;

//beta_old=1.570;
/*
  	homol_matrix A_B_TOOL;
  	homol_matrix tmp222;
  	  	homol_matrix tmp333;
	  homol_matrix A_B_T_base (0.709, -0.031, 1.026, -0.044, 2.769, -2.404);
	homol_matrix A_B_T_desire (0.709, -0.031, 1.026, 0.0,  M_PI/2, -M_PI/2);


	A_B_TOOL = !A_B_T_base * A_B_T_desire;

	tmp222 = A_B_T_desire; // A_B_T_base * A_B_TOOL;

	tmp333 = !tmp222 * A_B_T_desire;

	std::cout << tmp333 <<std::endl;


A_B_T_desire.get_xyz_euler_zyz(tablica);
for(int i=0; i<6; i++)
	printf("%f ,",tablica[i]);
printf("\n");


	A_B_TOOL.to_table(the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame);
	*/

/*
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][0]=cos(xalfa)*cos(xbeta)*cos(xgamma)-sin(xalfa)*sin(xgamma);
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][0]=-cos(xalfa)*cos(xbeta)*sin(xgamma)-sin(xalfa)*cos(xgamma);
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][0]=cos(xalfa)*sin(xbeta); the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][0]=xx;

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][1]=sin(xalfa)*cos(xbeta)*cos(xgamma)+cos(xalfa)*sin(xgamma);
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][1]=-sin(xalfa)*cos(xbeta)*sin(xgamma)+cos(xalfa)*cos(xgamma);
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][1]=sin(xalfa)*sin(xbeta); the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][1]=xy;

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][2]=-sin(xbeta)*cos(xgamma); the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][2]=sin(xbeta)*sin(xgamma);
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][2]=cos(xbeta); the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][2]=xz;
*/

// TOOL jednostkowy

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][0]=1; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][0]=0;
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][0]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][3]=0;

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][1]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][1]=1;
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][1]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][3]=0;

the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][2]=0; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][2]=0;
the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][2]=1; the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][3]=0.25;


/*

      the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
      // the_robot->ecp_command.instruction.get_type =  ARM_DEFINITION + ROBOT_MODEL_DEFINITION;
      // the_robot->ecp_command.instruction.set_type =  ARM_DEFINITION + ROBOT_MODEL_DEFINITION;
      the_robot->ecp_command.instruction.get_type =  ROBOT_MODEL_DEFINITION;
      the_robot->ecp_command.instruction.set_type =  ROBOT_MODEL_DEFINITION;
      the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
      the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
       the_robot->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
      the_robot->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
      the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
       the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
      the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
      the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
   */
/* } else {


      the_robot->ecp_command.instruction.instruction_type = lib::GET;
      the_robot->ecp_command.instruction.get_type = 0x04;
      the_robot->ecp_command.instruction.set_type = 0x04;
      the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
      the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
      the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
      the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
      the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
      the_robot->create_command ();
// }



*/
  return true;
}; // end: bool irp6ot_linear_generator::first_step ( )
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Generator prostoliniowy
bool seven_eye_run_linear::next_step (  ) {
 // Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
 // Funkcja zwraca false gdy koniec generacji trajektorii
 // Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
 // UWAGA: dzialamy na jednoelementowej liscie robotow
   int i; // licznik kolejnych wspolrzednych wektora [0..6]

// printf("w yoyek_linear_generator next_step\n");
   // Kontakt z MP
// if ((mp_comm_counter++)==100) {// by Y - lekka manianka
// mp_comm_counter=0; // 2004.02.25
clock_gettime( CLOCK_REALTIME , &s_time);
   if (check_and_null_trigger()) { // Koniec odcinka
//    ecp_t.set_ecp_reply (lib::TASK_TERMINATED);

     return false;
   }

   // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

   if(the_first)
   {
   printf("###################################\n");
   the_robot->ecp_command.instruction.instruction_type = lib::GET; //po sugestii Tomka
   the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;

   the_first=0;
the_second=1;


  nr++;
    return true;

   }
   if(!the_first)
   {
   	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;

 the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
   the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
}
if(the_second)
{
for (i=0; i<6; i++)
{

	pose[i][0]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	pose[i][1]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	pose[i][2]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	measure[i][0]=the_robot->reply_package.arm.pf_def.arm_coordinates[i]; //GDY NIE MA INFO Z KAM
	measure[i][1]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	measure[i][2]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	stearing[i][1]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	stearing[i][2]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	stearing_d[i][1]=0;
	stearing_d[i][2]=0;
}

}
nr++;

#ifdef PRINT
printf("%ld CRR =", nr);
for (i=0; i<6; i++)
{

printf("%f ",the_robot->reply_package.arm.pf_def.arm_coordinates[i]);

}
printf("\n");
#endif
// the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;


   // for (int j=0; j<3 ; j++)
   //       	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[3][j]+=0.001;

// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
   // (okreslenie kolejnego wezla interpolacji)

// itoa((sensor_m.begin())->second->image.sensor_union.force.rez[0], buf, 10);

for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			frame1[i][j]=(sensor_m.begin())->second->image.sensor_union.camera.frame[4*i+j];

//WYREM2DEBUG


valid_measure=(int)frame1[3][3];

#ifdef PRINT
for(int i=0; i<4; i++)
{
		for(int j=0; j<4; j++)
			std::cout << frame1[i][j] <<' ';
		std::cout <<std::endl;
}
#endif
//UWAGA - ZAMIANA OSI X I Y

if(!the_second)
{
	//^{0}T_{G}=^{0}T_{C}*^{C}T_{G}
	measure[0][0]=0.001*frame1[1][3]+0.97-0.007;
	measure[1][0]=0.001*frame1[0][3]-0.025+0.008;
	measure[2][0]=-0.001*frame1[2][3]+1.2-0.09;  // -0.1na poz kostki
	frame1[0][3]=measure[0][0];
	frame1[1][3]=measure[1][0];
	frame1[2][3]=measure[2][0];
	//^{0}T_{P}=^{0}T_{G}*^{G}T_{P} //0.1 na Y - korekta kinemtyczna
	measure[0][0]=frame1[1][0]*(x2g)+frame1[1][1]*(0)+frame1[1][2]*0+frame1[0][3];
	measure[1][0]=frame1[0][0]*(x2g)+frame1[0][1]*(0)+frame1[0][2]*0+frame1[1][3];
//	measure[2][0]=frame1[2][0]*(x2g)+frame1[2][1]*0.+frame1[2][2]*0+frame1[2][3];
//	printf("%f %f %f\n",frame1[1][0]*(0.1)+frame1[1][1]*0+frame1[1][2]*0,frame1[1][3],measure[1][0]);
}


if(fabs(measure[0][1]-measure[0][0])<=0.005 && fabs(measure[1][1]-measure[1][0])<=0.005)
{
	steps2switch++;
}
else
{
	steps2switch=0;
}


if(steps2switch>30)
phase=1;


if(phase)
{
	x2g+=0.001;
}

if(x2g>0) x2g=0;

//odjazd

if(fabs(measure[0][1]-measure[0][0])>=0.050 || fabs(measure[1][1]-measure[1][0])>=0.050 || fabs(measure[0][1]-measure[0][0])>=0.050)
{
	x2g=-0.1;
	phase=0;
	steps2switch=0;
}


gammax=atan2(frame1[2][1],frame1[2][2]);
beta=atan2(-frame1[2][0],sqrt(frame1[0][0]*frame1[0][0]+frame1[1][0]*frame1[1][0]));
alfa=atan2(frame1[1][0],frame1[0][0]);


if(alfa>M_PI/2) alfa-=M_PI/2;
	alfa*=-1;



if(gammax>0) gammax=0;

#ifdef PRINT
std::cout <<"alfa="<<180*alfa/M_PI<<"     " <<alfa<<std::endl;
std::cout <<"beta="<<180*beta/M_PI<<"     " <<beta<<std::endl;
std::cout <<"gamma="<<180*gammax/M_PI<<"     " <<gammax<<std::endl;
#endif

if(beta<0) beta*=-1;
/*
 if(alfa>M_PI/2) alfa=M_PI/2;

 	measure[3][0]=-1*alfa;
*/
//WYREM2DEBUG

//measure[0][0]=;
if(!the_second)
{
	//measure[1][0]=0.140;
	measure[3][0]=alfa;
	measure[4][0]=1.57-beta;
	measure[5][0]=3.14+gammax-0.785;
}
/*
gammax=atan2(Rckk[3][2],Rckk[3][3]);
 	beta=atan2(-Rckk[3][1],sqrt(Rckk[1][1]*Rckk[1][1]+Rckk[2][1]*Rckk[2][1]));
 	alfa=atan2(Rckk[2][1],Rckk[1][1]);
 */
 /*
	gammax=180*gammax/M_PI;
 	beta=180*beta/M_PI;
 	alfa=180*alfa/M_PI;
 */
 /*
 	printf("ECP: alfa=%f, beta=%f, gammax=%f\n", alfa, beta, gammax);
	printf("VALID: %d\n", valid_measure);
*/


/*
for(int i=0; i<4; i++)
{
		for(int j=0; j<4; j++)
			printf("%f ",frame1[i][j]);
		printf("\n");
}
*/

//skladajcy dziubek
/*
if(nr==30)
measure[1][0]-=0.001;

if(nr==31)
measure[1][0]-=0.007;

if(nr>=32)
measure[1][0]+=0.023;
*/

//if(nr==30)
//measure[1][0]-=0.001;

//wystarczy taki dziubek
/*
if(nr==31)
measure[1][0]-=0.007;

if(nr>=32)
measure[1][0]+=0.002;
*/

/*
if(nr==30)
measure[1][0]-=0.03;

if(nr==31)
measure[1][0]-=0.03;

if(nr>=32)
measure[1][0]+=0.03;
*/



 // krancowki
#ifdef PRINT
printf("%ld MSR =", nr);
for (i=0; i<6; i++)
{

printf("%f ",measure[i][0]);

}
printf("\n");
#endif

for(int i=0; i<6; i++)
 {
 	 if(measure[i][0]<measure_border_d[i])
 		measure[i][0]=measure_border_d[i];

 	if(measure[i][0]>measure_border_u[i])
 		measure[i][0]=measure_border_u[i];
 }
#ifdef PRINT
printf("%ld MSQ =", nr);
for (i=0; i<6; i++)
{

printf("%f ",measure[i][0]);

}
printf("\n");
#endif

//printf("%d ", nr);

//printf("%f ",measure[1][0]);

 for(int i=0; i<6; i++)
 stearing[i][0]=stearing[i][1] + 0.5*(measure[i][0]-stearing[i][1]);
 //y - kp=0.5
//alfa - kp=0.05

stearing[2][0]=stearing[2][1] + 0.2*(measure[2][0]-stearing[2][1]);
stearing[3][0]=stearing[3][1] + 0.1*(measure[3][0]-stearing[3][1]);
stearing[4][0]=stearing[4][1] + 0.05*(measure[4][0]-stearing[4][1]);
stearing[5][0]=stearing[5][1] + 0.05*(measure[5][0]-stearing[5][1]);



#ifdef PRINT
 printf("%ld STR =", nr);
for (i=0; i<6; i++)
{

printf("%f ",stearing[i][0]);

}
printf("\n");
#endif

// printf("%f ",stearing[1][0]);

 // roznica w kroku -> docelowo predkosc
 for(int i=0; i<6; i++)
 	stearing_d[i][0]=stearing[i][0]-stearing[i][1];

 // roznica w 2 krokach -> docelowo przyspieszenie
 for(int i=0; i<6; i++)
 	stearing_d2[i][0]=stearing_d[i][0]-stearing_d[i][1];

//printf("DEB X %f %f    Y %f %f\n",stearing_d[0][0],stearing_d2[0][0], stearing_d[1][0],stearing_d2[1][0]);
//printf("%f ",stearing_d[1][0]);
//printf("%f ",stearing_d2[1][0]);

 debug=0;
 //ograniczenie przyspieszenia (opoznienie zalatwia regulator proporcjonalny)
for(int i=0; i<6; i++)
 {
 	if((fabs(stearing_d2[i][0])>=d2_u_max[i]) && (fabs(stearing_d[i][0])>=fabs(stearing_d[i][1])) )
 	{
 		if(stearing_d[i][0]>=0)
 		{
 			stearing_d[i][0]=stearing_d[i][1]+d2_u_max[i];
 			stearing[i][0]=stearing[i][1]+stearing_d[i][0];
 		//	if (i==1) debug+=1;
 		}
 		else
		if(stearing_d[i][0]<0)
 		{
 			stearing_d[i][0]=stearing_d[i][1]-d2_u_max[i];
 			stearing[i][0]=stearing[i][1]+stearing_d[i][0];
 		//	if (i==1) debug+=10;
 		}
	}
}


 //ograniczenie predkosci
 for(int i=0; i<6; i++)
 {
 //	if(abs(stearing_d[i][0])>=0)
 //	{
 		if(stearing_d[i][0]>=d_u_max[i])
 		{
 			stearing[i][0]=stearing[i][1]+d_u_max[i];
 		//	if (i==1) debug+=100;
 		}
// 	}
//	 else
//	{
 		if(stearing_d[i][0]<=-1*d_u_max[i])
 		{
 			stearing[i][0]=stearing[i][1]-d_u_max[i];
 		//	if (i==1) debug+=1000;
 		}
// 	}
 }

 //ograniczenie przyspieszenia
for(int i=0; i<6; i++)
{
	stearing_d2[i][0]=stearing_d[i][0]-stearing_d[i][1];
//	if (i==1) debug+=10000;
}



#ifdef PRINT
 printf("%ld STQ =", nr);
for (i=0; i<6; i++)
{

printf("%f ",stearing[i][0]);

}
printf("\n");


#endif

//printf("%f ",stearing[1][0]);

// chuchamy na zimne - krancowki na sterowaniu
  for(int i=0; i<6; i++)
 {
 	 if(stearing[i][0]<measure_border_d[i])
 		stearing[i][0]=measure_border_d[i];

 	if(stearing[i][0]>measure_border_u[i])
 		stearing[i][0]=measure_border_u[i];
 }


#ifdef PRINT
 printf("%ld STR0=", nr);
 	for (int i=0; i<6 ; i++)
       printf("%f  ",stearing[i][0]);
	printf("\n");
#endif

#ifdef PRINTA
printf("%d STR1=", nr);
	for (int i=0; i<6 ; i++)
       printf("%f  ",stearing[i][1]);
	printf("\n");
#endif

//printf("%f %d\n",stearing[1][0],debug);

 /*
if(the_first)
{
the_first=0;
the_second=1;
printf("XXXXXXXXXXXXXXXXFIRST\n");
}
else*/
if(the_second)
{


for (i=0; i<6; i++)
{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=stearing[i][0];

		stearing[i][2]=stearing[i][1];
		stearing[i][1]=stearing[i][0];
		stearing_d[i][2]=stearing_d[i][1];
		stearing_d[i][1]=stearing_d[i][0];
		measure[i][2]=measure[i][1];
		measure[i][1]=measure[i][0];
}
	//beta_old=beta;
	/*CHYCHAMY NA ZIMNE - WYLACZENIE NIESTEROWANYCH OSI*/
	for (i=0; i<0; i++)
      {
      	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
      }
      for (i=6; i<6; i++)
      {
      	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
      }
	/*******************************************************/
#ifdef PRINTA
printf("%d ARM 2d= ", nr);
 	for (int i=0; i<6 ; i++)
       printf("%f  ",the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]);

	printf("\n");

#endif

the_second=0;
the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate=the_robot->reply_package.arm.pf_def.gripper_coordinate;
(sensor_m.begin())->second->base_period=1;
 (sensor_m.begin())->second->current_period=1;
}
else
{
      for (i=0; i<6; i++)
      {
      	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=stearing[i][0];

		stearing[i][2]=stearing[i][1];
		stearing[i][1]=stearing[i][0];
		stearing_d[i][2]=stearing_d[i][1];
		stearing_d[i][1]=stearing_d[i][0];
		measure[i][2]=measure[i][1];
		measure[i][1]=measure[i][0];
	}

	//beta_old=beta;

	/*CHYCHAMY NA ZIMNE - WYLACZENIE NIESTEROWANYCH OSI*/
	for (i=0; i<0; i++)
      {
      	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
      }
      for (i=6; i<6; i++)
      {
      	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
      }
	/*******************************************************/
	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate=the_robot->reply_package.arm.pf_def.gripper_coordinate;
#ifdef PRINTA
	printf("%d ARM= ", nr);
 	for (int i=0; i<6 ; i++)
       printf("%f  ",the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]);
	printf("\n");


#endif

}
clock_gettime( CLOCK_REALTIME , &crr_time);




clock_gettime( CLOCK_REALTIME , &e_time);
// printf( "ECP= %f %f %f\n",(double)(e_time.tv_nsec), (double)(crr_time.tv_nsec), (double)(s_time.tv_nsec));

   // skopiowac przygotowany rozkaz dla EDP do bufora wysylkowego
// the_robot->create_command ();
   return true;
}; // end: bool irp6ot_linear_generator::next_step ( )

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

