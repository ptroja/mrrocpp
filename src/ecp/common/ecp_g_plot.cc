// -------------------------------------------------------------------------
//                            ecp_gen_plot.cc
//            Effector Control Process (ECP) - rysowanie
// 			Funkcje do tworzenia procesow ECP z rysowaniem
// 			Ostatnia modyfikacja: 01.06.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_plot.h"

#include "lib/mathtr.h"

y_simple_generator::y_simple_generator(ecp_task& _ecp_task, int step):
        ecp_generator (_ecp_task, true)
{
    step_no = step;
};

static FILE *file;

bool y_simple_generator::first_step ( )
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


    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
    the_robot->EDP_data.set_type = ARM_DV;

    the_robot->EDP_data.set_arm_type =  XYZ_EULER_ZYZ;			// orientacja euler'owska
    the_robot->EDP_data.get_arm_type =  XYZ_EULER_ZYZ;

    the_robot->EDP_data.motion_type = ABSOLUTE;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;




    return true;
}
; // end: bool y_simple_generator::first_step (map <SENSOR_ENUM, sensor*>& sensor_m, robot& the_robot )
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_simple_generator::next_step ( )
{
    // zmienne wykorzystywane przy rysowaniu
    static double pozycja[3];
    double temp, roznica;
    static double nowa_pozycja[3];
    int znacz = 1;
    static int ruch;
    //const double _dlugosc = 0.08;


    //	struct timespec start[9];
    if (ecp_t.pulse_check())
    {
        ecp_t.mp_buffer_receive_and_send ();
        return false;
    }




    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.get_type = NOTHING_DV;
    the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;



    //	printf("---> %d --->", node_counter);

    // zapisanie poczatkowych wspolrzednych XYZ
    if(node_counter < 2)
    {
        ruch =0;
        for(int i; i<3; i++)
        {
            pozycja[i] = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
            nowa_pozycja[i] = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
        }
        //	printf("-=%f - %f - %f=-\n", pozycja[0], pozycja[1], pozycja[2]);

        // zatrzymanie robota w miejscu
        for (int i = 0; i < 6; i++)
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
        the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;
    }

    // do zapisu logfile'a z ruchu
    file = fopen("logfile.txt", "a+");

    // rysowanie kwadratu
    if(znacz == 0)
    {
        if(nowa_pozycja[0] - pozycja[0] <= 0.04 && ruch == 0)
        {
            nowa_pozycja[0] += 0.0002;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] += 0.0002;
        }
        else if(nowa_pozycja[1] - pozycja[1] <= 0.04 && ruch == 0)
        {
            nowa_pozycja[1] += 0.0002;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += 0.0002;
            if(nowa_pozycja[1] - pozycja[1]>0.04 )
                ruch = 2;
        }
        else if(nowa_pozycja[0] - pozycja[0] >= 0.0 && ruch == 2)
        {
            nowa_pozycja[0] -= 0.0002;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] -= 0.0002;
        }
        else if(nowa_pozycja[1] - pozycja[1] >= 0.0 && ruch == 2)
        {
            nowa_pozycja[1] -= 0.0002;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] -= 0.0002;
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
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] += 0.0001;

            akt_pozycja = nowa_pozycja[0] - promien;
            // przesuwamy jednoczesnie w bok
            temp = sqrt(pow(promien, 2) - pow(akt_pozycja, 2));
            if ( !(temp == temp) )
                temp = 0.001;

            roznica = nowa_pozycja[1] - pozycja[1];
            //nowa_pozycja[1] -= (promien - temp) - abs(roznica);
            nowa_pozycja[1] -= (temp) - fabs(roznica);

            //			assert(((promien - temp) - abs(roznica)) > 0.01 || ((promien - temp) - abs(roznica)) < -0.01);

            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] -= (temp) - fabs(roznica);

            printf("jeden: %f; %f ->%f, ->%f; r=%f\n", nowa_pozycja[0], nowa_pozycja[1], akt_pozycja, temp, roznica);
            //			printf("-> %f\n", (temp) - abs(roznica));
            fprintf(file, "jeden: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] >= promien && nowa_pozycja[0] - pozycja[0]< (2*promien) && ruch == 0)
        {
            // posuwamy do przodu
            nowa_pozycja[0] += 0.0001;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] += 0.0001;

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

            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += (temp) + fabs(roznica);

            printf("dwa: %f; %f ->%f, ->%f; r=%f\n", nowa_pozycja[0], nowa_pozycja[1], akt_pozycja, temp, roznica);
            fprintf(file, "dwa: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] >= promien && ruch == 2)
        {


            printf ("-=KONIEC=-");
            ecp_t.ecp_termination_notice();
            return false;


            // posuwamy do tylu
            nowa_pozycja[0] -= 0.0001;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] -= 0.0001;
            // przesuwamy jednoczesnie w bok
            temp = sqrt(pow(promien, 2) - pow(akt_pozycja, 2));
            if ( !(temp == temp) )
                temp = 0.001;

            roznica = nowa_pozycja[1] - pozycja[1];
            nowa_pozycja[1] += (temp) - fabs(roznica);

            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += (temp) - fabs(roznica);
            printf("trzy: %f; %f ->%f, ->%f; r=%f\n", nowa_pozycja[0], nowa_pozycja[1], akt_pozycja, temp, roznica);
            fprintf(file, "trzy: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] < promien && nowa_pozycja[0] - pozycja[0] > 0 && ruch == 2)
        {
            // posuwamy do tylu
            nowa_pozycja[0] -= 0.0001;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] -= 0.0002;
            // przesuwamy jednoczesnie w bok
            temp = sqrt(pow(promien, 2) - pow(akt_pozycja, 2));
            if ( !(temp == temp) )
                temp = 0.001;

            roznica = nowa_pozycja[1] - pozycja[1];
            nowa_pozycja[1] -= (temp) - fabs(roznica);

            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] -= (temp) - fabs(roznica);
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
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] += 0.0001;
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

            //			the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += (0.02 -temp) - (nowa_pozycja[1] - pozycja[1]);
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += (0.02 -temp)- roznica;
            //			if(nowa_pozycja[0] - pozycja[0] >= 0.02)
            //				return 99;

            printf("jeden: %f; %f  <%f; %f>\n", nowa_pozycja[0], nowa_pozycja[1], the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0],
                   the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1]);

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
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] += 0.0001;
            temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]) -0.04/2, 2));
            //			nowa_pozycja[1] += (temp - 0.02) - (nowa_pozycja[1] - pozycja[1] - 0.02);
            roznica = nowa_pozycja[1]- pozycja[1];
            nowa_pozycja[1] += (temp - 0.04/2) - roznica;
            //			the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += (temp - 0.02) - (nowa_pozycja[1] - pozycja[1] - 0.02);
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += (temp - 0.04/2) - roznica;
            if(nowa_pozycja[0] - pozycja[0] >= 0.04)
            {
                ruch = 2;
                fprintf(file, "\nznacznik_2\n");
      
                ruch = 99;
            }
            printf("dwa: %f; %f  <%f; %f>\n", nowa_pozycja[0], nowa_pozycja[1], the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0],
                   the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1]);
            fprintf(file, "dwa: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] >= (0.04/2) && ruch == 2)
            //		else if(nowa_pozycja[0] - pozycja[0] <= 0.04 + 0.005 && ruch == 0 && nowa_pozycja[0] - pozycja[0] >=0.04)
        {
            nowa_pozycja[0] -= 0.0001;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] -= 0.0001;
            temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]) - 0.04/2, 2));
            //			temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]), 2));

            roznica = nowa_pozycja[1]- pozycja[1];
            nowa_pozycja[1] -= temp - roznica;
            //			printf("__trzy: %f; %f  <%f; %f>\n", nowa_pozycja[0], nowa_pozycja[1], the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0],
            //		the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1]+temp);
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += temp;
            printf("trzy: %f; %f  <%f; %f>\n", nowa_pozycja[0], nowa_pozycja[1], the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0],
                   the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1]);
            fprintf(file, "trzy: %f; %f\n", nowa_pozycja[0], nowa_pozycja[1]);
        }
        else if(nowa_pozycja[0] - pozycja[0] < (0.04/2) && nowa_pozycja[0] - pozycja[0] > 0 && ruch == 2)
            //		else if(nowa_pozycja[0] - pozycja[0] <= 0.04/2 && ruch == 2)
        {
            nowa_pozycja[0] -= 0.0002;
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] -= 0.0002;
            temp = sqrt(pow(0.04/2, 2) - pow((nowa_pozycja[0] - pozycja[0]), 2));
            //			nowa_pozycja[1] += -temp - (nowa_pozycja[1] - pozycja[1]);
            nowa_pozycja[1] += -temp;
            //			the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += -temp - (nowa_pozycja[1] - pozycja[1]);
            the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] += -temp;
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
        ecp_t.ecp_termination_notice();
        return false;
    }





    return true;
};
