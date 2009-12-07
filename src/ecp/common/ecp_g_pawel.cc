// -------------------------------------------------------------------------
//                            ecp_g_pawel.cc
//            Effector Control Process (lib::ECP) - rysowanie
// 			Funkcje do tworzenia procesow ECP z rysowaniem
// 			Ostatnia modyfikacja: 01.06.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_pawel.h"
#include "ecp_mp/ecp_mp_s_pawel.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

#define XMAX 768.0
#define YMAX 574.0
#define ZMAX 350.0

#define XCENTER XMAX/2
#define YCENTER YMAX/2
#define ZCENTER 20.0
#define MAXDISTANCE 25.0

#define PMIN 0.0
#define Z_AMPLIFIER 5.0

#define FRICTION 40000.0
#define INERTIA 8000.0

#define min(x,y) ( x<y ? x : y )

// uchyby:
double ex,ey,ez;  // uchyby
double ux,uy,uz;  // sterowania
double ux1,uy1,uz1; // poprzednie sterowania
double start; // poczatek dzialania programu sekundach od epoki
struct timespec time_start, time_tmp;
FILE* research;


pawel::pawel(common::task::task& _ecp_task, int step):
	generator (_ecp_task)
{

    step_no = step;
    state = 0; //oczekiwanie na pilke

    ex  =ey  =ez  =0.0;
    ux1=uy1=uz1=0.0;
    research = fopen( "/net/mieszko/home/pnajgebauer/mrrocpp2/data/res.txt", "w" );

}

pawel::~pawel()
{

    fclose( research );

}

bool pawel::first_step ( )
{

    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 4;
    clock_gettime( CLOCK_REALTIME , &time_start);
    //	start = (double)time_start.tv_sec + ((double)(time_start.tv_nsec))/1000000000.0;


    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV;
    the_robot->ecp_command.instruction.set_type = ARM_DV;

    the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;

    the_robot->EDP_data.motion_type = lib::RELATIVE;
     the_robot->EDP_data.next_interpolation_type = lib::MIM;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;



    return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool pawel::next_step ( )
{

    int i;
    double x,y,z,t,vx,vy,v,tmp;

    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.set_type = ARM_DV;
    the_robot->ecp_command.instruction.get_type = NOTHING_DV;
    the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;


    x = sensor_m[lib::SENSOR_PAWEL]->image.sensor_union.ball.x;
    y = sensor_m[lib::SENSOR_PAWEL]->image.sensor_union.ball.y;
    z = sensor_m[lib::SENSOR_PAWEL]->image.sensor_union.ball.z;
    i = sensor_m[lib::SENSOR_PAWEL]->image.sensor_union.ball.nr;
    /*	t = (double) sensor_m[SENSOR_PAWEL]->image.sensor_union.ball.ts.tv_sec +
    		((double)sensor_m[SENSOR_PAWEL]->image.sensor_union.ball.ts.tv_nsec)/1000000000 - start;*/
    clock_gettime( CLOCK_REALTIME , &time_tmp);
    t = (double)(time_tmp.tv_sec - time_start.tv_sec) + ((double)(time_tmp.tv_nsec - time_start.tv_nsec))/1000000000.0;

    if( x>=XMAX || x<=PMIN || y>=YMAX || y<=PMIN || z>=ZMAX || z<=PMIN )
    {

        state = 0;

    }
    else
    {

        state = 1;

    }

    // przeliczenie promienia pilki na odleglosc

    z = 3761.0/(z + 67.1) - 8.537;

    // przeliczenie z uwzglednieniem geometrii:

    tmp = sqrt( (XCENTER-x)*(XCENTER-x) + (YCENTER-y)*(YCENTER-y) )/25;
    z = sqrt( z*z - tmp*tmp );

    switch(state)
    {

    case 0 :
    default:

        // oczekiwanie na koordynaty
        // wygaszanie uchybow

        v = 0.0;

        ex  = 0.9*ex;
        ey  = 0.9*ey;
        ez  = 0.9*ez;

        break;

    case 1 :

        vx = XCENTER - x - ex;
        vy = YCENTER - y - ey;
        v = sqrt( vx*vx + vy*vy );


        // wyznaczenie uchybow

        ex = XCENTER-x;
        ey = YCENTER-y;
        ez = z- min( ZCENTER + v/3.0 , MAXDISTANCE);
        //ez = z - ZCENTER;

        printf ("[GENERATOR] z: %f v: %f\n",z,v);

        break;
    }

    //		printf ("[GENERATOR] %f\n",t);

    // zapisanie poprzednich sterowan

    ux1 = ux;
    uy1 = uy;
    uz1 = uz;

    // wyznaczenie nowych sterowan

    ux = (double)((ex + ux1*INERTIA)/(INERTIA + FRICTION));
    uy = (double)((ey + uy1*INERTIA)/(INERTIA + FRICTION));
    uz = (double)(Z_AMPLIFIER*(ez + uz1*INERTIA)/(INERTIA + FRICTION));

    //		the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] = 0.0;
    //		the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] = 0.0;
    //		the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] = 0.0;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] = ux;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] = uy;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] = uz;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] = 0.0;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] = 0.0;
    the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5] = 0.0;
    the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;

    // zapis do pliku

    fprintf( research, "%f %f %f %f %f %f %f\n", t, ex, ey, ez, ux, uy, uz );



    return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
