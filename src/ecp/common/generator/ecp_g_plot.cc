// -------------------------------------------------------------------------
//                            ecp_gen_plot.cc
//            Effector Control Process (lib::ECP) - rysowanie
// 			Funkcje do tworzenia procesow ECP z rysowaniem
// 			Ostatnia modyfikacja: 01.06.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/generator/ecp_g_plot.h"

#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

y_simple::y_simple(common::task::task& _ecp_task, int step):
	generator (_ecp_task)
{
    step_no = step;
}

static FILE *file;

bool y_simple::first_step ( )
{


    run_counter = 0;
    second_step = false;
    for (int j=0; j<3; j++)
        for (int i=0; i<4; i++)
            previous_frame[j][i]=0;

    for (int i=0; i<6; i++)
        delta[i]=0.0;


    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;


    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->ecp_command.instruction.set_type = ARM_DV;

    the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;			// orientacja euler'owska
    the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;

    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
     the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
    the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;




    return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_simple::next_step ( )
{
    // zmienne wykorzystywane przy rysowaniu
    static double pozycja[3];
    double temp, roznica;
    static double nowa_pozycja[3];
    int znacz = 1;
    static int ruch;
    //const double _dlugosc = 0.08;


    //	struct timespec start[9];
    if (check_and_null_trigger())
    {
        return false;
    }




    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->ecp_command.instruction.set_type = ARM_DV;
    the_robot->ecp_command.instruction.get_type = NOTHING_DV;
    the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;



    //	printf("---> %d --->", node_counter);

    // zapisanie poczatkowych wspolrzednych XYZ
    if(node_counter < 2)
    {
        ruch =0;
        for(int i = 0; i<3; i++)
        {
            pozycja[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
            nowa_pozycja[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
        }
        //	printf("-=%f - %f - %f=-\n", pozycja[0], pozycja[1], pozycja[2]);

        // zatrzymanie robota w miejscu
        for (int i = 0; i < 6; i++)
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
        the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;
    }

    // do zapisu logfile'a z ruchu
    file = fopen("logfile.txt", "a+");

    // rysowanie kwadratu
    if(znacz == 0)
    {
        if(nowa_pozycja[0] - pozycja[0] <= 0.04 && ruch == 0)
        {
            nowa_pozycja[0] += 0.0002;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] += 0.0002;
        }
        else if(nowa_pozycja[1] - pozycja[1] <= 0.04 && ruch == 0)
        {
            nowa_pozycja[1] += 0.0002;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += 0.0002;
            if(nowa_pozycja[1] - pozycja[1]>0.04 )
                ruch = 2;
        }
        else if(nowa_pozycja[0] - pozycja[0] >= 0.0 && ruch == 2)
        {
            nowa_pozycja[0] -= 0.0002;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] -= 0.0002;
        }
        else if(nowa_pozycja[1] - pozycja[1] >= 0.0 && ruch == 2)
        {
            nowa_pozycja[1] -= 0.0002;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] -= 0.0002;
            if(nowa_pozycja[1] - pozycja[1] < 0.0)
                ruch = 99;
        }
    }

    // rysowanie kola
    else if(znacz == 1)
    {
        double promien = 0.02;
        double akt_pozycja = 0;

        if(nowa_pozycja[0] - pozycja[0]< (promien) && ruch == 0)
        {
            // posuwamy do przodu
            nowa_pozycja[0] += 0.0001;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] += 0.0001;

            akt_pozycja = nowa_pozycja[0] - promien;
            // przesuwamy jednoczesnie w bok
            temp = sqrt(pow(promien, 2) - pow(akt_pozycja, 2));
            if ( !(temp == temp) )
                temp = 0.001;

            roznica = nowa_pozycja[1] - pozycja[1];
            //nowa_pozycja[1] -= (promien - temp) - abs(roznica);
            nowa_pozycja[1] -= (temp) - fabs(roznica);

            //			assert(((promien - temp) - abs(roznica)) > 0.01 || ((promien - temp) - abs(roznica)) < -0.01);

            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] -= (temp) - fabs(roznica);

            printf("jeden: %f; %f ->%f, ->%f; r=%f\n", nowa_pozycja[0], nowa_pozycja[1], akt_pozycja, temp, roznica);
            //			printf("-> %f\n", (temp) - abs(roznica));
            fprintf(file, "jeden: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] >= promien && nowa_pozycja[0] - pozycja[0]< (2*promien) && ruch == 0)
        {
            // posuwamy do przodu
            nowa_pozycja[0] += 0.0001;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] += 0.0001;

            akt_pozycja = nowa_pozycja[0] - promien;
            // przesuwamy jednoczesnie w bok
            temp = sqrt(pow(promien, 2) - pow(akt_pozycja, 2));
            if ( !(temp == temp) )
            {
                temp = 0.001;
                //printf("\t\t tutaj! %f\n", temp);
            }

            roznica = nowa_pozycja[1] - pozycja[1];
            nowa_pozycja[1] += (temp) + fabs(roznica);
            if(nowa_pozycja[0] - pozycja[0] >= 0.04)
            {
                ruch = 2;
            }

            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += (temp) + fabs(roznica);

            printf("dwa: %f; %f ->%f, ->%f; r=%f\n", nowa_pozycja[0], nowa_pozycja[1], akt_pozycja, temp, roznica);
            fprintf(file, "dwa: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] >= promien && ruch == 2)
        {


            printf ("-=KONIEC=-");

            return false;


            // posuwamy do tylu
            nowa_pozycja[0] -= 0.0001;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] -= 0.0001;
            // przesuwamy jednoczesnie w bok
            temp = sqrt(pow(promien, 2) - pow(akt_pozycja, 2));
            if ( !(temp == temp) )
                temp = 0.001;

            roznica = nowa_pozycja[1] - pozycja[1];
            nowa_pozycja[1] += (temp) - fabs(roznica);

            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += (temp) - fabs(roznica);
            printf("trzy: %f; %f ->%f, ->%f; r=%f\n", nowa_pozycja[0], nowa_pozycja[1], akt_pozycja, temp, roznica);
            fprintf(file, "trzy: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] < promien && nowa_pozycja[0] - pozycja[0] > 0 && ruch == 2)
        {
            // posuwamy do tylu
            nowa_pozycja[0] -= 0.0001;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] -= 0.0002;
            // przesuwamy jednoczesnie w bok
            temp = sqrt(pow(promien, 2) - pow(akt_pozycja, 2));
            if ( !(temp == temp) )
                temp = 0.001;

            roznica = nowa_pozycja[1] - pozycja[1];
            nowa_pozycja[1] -= (temp) - fabs(roznica);

            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] -= (temp) - fabs(roznica);
            if(nowa_pozycja[0] - pozycja[0] <= 0)
            {
                ruch = 99;
            }
            //printf("cztery: %f\n", nowa_pozycja[0]);
            printf("cztery: %f; %f ->%f, ->%f; r=%f\n", nowa_pozycja[0], nowa_pozycja[1], akt_pozycja, temp, roznica);
            fprintf(file, "cztery: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        fclose(file);
    }
    if(znacz == 2)
    {
        if(nowa_pozycja[0] - pozycja[0]< (0.04/2) && ruch == 0)
            //		if(nowa_pozycja[0] - pozycja[0] < 0.04/2 && ruch == 0)
        {
            nowa_pozycja[0] += 0.0001;
            if(nowa_pozycja[0] - pozycja[0]>0.04/2)
            {

                return true;
            }
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] += 0.0001;
            temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]), 2));
            if(temp ==0)
            {

                return true;
            }
            //			nowa_pozycja[1] += (0.02 - temp) - (nowa_pozycja[1] - pozycja[1]);
            roznica = nowa_pozycja[1]- pozycja[1];
            nowa_pozycja[1] += (0.04/2 - temp) - roznica;
            if((0.04/2 - temp) - roznica >= 0.001)
            {

                return true;
            }

            //			the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += (0.02 -temp) - (nowa_pozycja[1] - pozycja[1]);
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += (0.02 -temp)- roznica;
            //			if(nowa_pozycja[0] - pozycja[0] >= 0.02)
            //				return 99;

            printf("jeden: %f; %f  <%f; %f>\n", nowa_pozycja[0], nowa_pozycja[1], the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0],
                   the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1]);

            fprintf(file, "jeden: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] >= (0.04/2) && nowa_pozycja[0] - pozycja[0]< 0.04 && ruch == 0)
            //		else if(nowa_pozycja[0] - pozycja[0] < 0.04 && ruch == 0)
        {
            nowa_pozycja[0] += 0.0001;
            if(nowa_pozycja[0] - pozycja[0] > 0.04)
            {
                ruch = 2;
                fprintf(file, "\nznacznik_1\n");

                return true;
            }
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] += 0.0001;
            temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]) -0.04/2, 2));
            //			nowa_pozycja[1] += (temp - 0.02) - (nowa_pozycja[1] - pozycja[1] - 0.02);
            roznica = nowa_pozycja[1]- pozycja[1];
            nowa_pozycja[1] += (temp - 0.04/2) - roznica;
            //			the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += (temp - 0.02) - (nowa_pozycja[1] - pozycja[1] - 0.02);
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += (temp - 0.04/2) - roznica;
            if(nowa_pozycja[0] - pozycja[0] >= 0.04)
            {
                ruch = 2;
                fprintf(file, "\nznacznik_2\n");

                ruch = 99;
            }
            printf("dwa: %f; %f  <%f; %f>\n", nowa_pozycja[0], nowa_pozycja[1], the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0],
                   the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1]);
            fprintf(file, "dwa: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] >= (0.04/2) && ruch == 2)
            //		else if(nowa_pozycja[0] - pozycja[0] <= 0.04 + 0.005 && ruch == 0 && nowa_pozycja[0] - pozycja[0] >=0.04)
        {
            nowa_pozycja[0] -= 0.0001;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] -= 0.0001;
            temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]) - 0.04/2, 2));
            //			temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]), 2));

            roznica = nowa_pozycja[1]- pozycja[1];
            nowa_pozycja[1] -= temp - roznica;
            //			printf("__trzy: %f; %f  <%f; %f>\n", nowa_pozycja[0], nowa_pozycja[1], the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0],
            //		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1]+temp);
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += temp;
            printf("trzy: %f; %f  <%f; %f>\n", nowa_pozycja[0], nowa_pozycja[1], the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0],
                   the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1]);
            fprintf(file, "trzy: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] < (0.04/2) && nowa_pozycja[0] - pozycja[0] > 0 && ruch == 2)
            //		else if(nowa_pozycja[0] - pozycja[0] <= 0.04/2 && ruch == 2)
        {
            nowa_pozycja[0] -= 0.0002;
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] -= 0.0002;
            temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]), 2));
            //			nowa_pozycja[1] += -temp - (nowa_pozycja[1] - pozycja[1]);
            nowa_pozycja[1] += -temp;
            //			the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += -temp - (nowa_pozycja[1] - pozycja[1]);
            the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] += -temp;
            if(nowa_pozycja[0] - pozycja[0] > 0.04)
            {

                ruch = 99;
            }
            fprintf(file, "cztery: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        fclose(file);
    }
    else if(ruch == 99)
    {
        printf ("-=KONIEC=-");

        return false;
    }

    return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
