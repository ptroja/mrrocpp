// -------------------------------------------------------------------------
//                            ecp_g_smooth.cc
//            Effector Control Process (ECP) - smooth generator
// Generator nie wykorzystujacy informacji o czasie ruchu
// autor: Przemek Pilacinski
// Ostatnia modyfikacja: 2007r.
// -------------------------------------------------------------------------


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth.h"
#include <fstream>
#include <string.h>
#include "ecp/common/ecp_smooth_taught_in_pose.h"
//#include "lib/y_math.h"

void ecp_smooth_generator::generate_next_coords (void)
{
    //funkcja obliczajaca polozenie w danym makrokroku

    int i;
    double tk=10*STEP;

    for(i=0; i<MAX_SERVOS_NR; i++)
    {
        if(node_counter<przysp[i])
        { //pierwszy etap
            if(v_p[i]<=v_r[i])
            { //przyspieszanie w pierwszym etapie
                if(debug)
                {
                    printf("%.16f ", ( start_position[i] + k[i]*(node_counter*v_p[i]*tk +
                                       node_counter*node_counter*a_r[i]*tk*tk/2)));
                    if(i==7)
                        printf("\n");
                }

                next_position[i] = start_position[i] +
                                   k[i]*(node_counter*v_p[i]*tk + node_counter*node_counter*a_r[i]*tk*tk/2);
            }
            else
            { //hamowanie w pierwszym etapie
                if(debug)
                {
                    printf("%.16f", (start_position[i] +
                                     k[i]*((node_counter*tk*v_p[i]) -
                                           (node_counter*node_counter*tk*tk*a_r[i]/2))));
                    if(i==7)
                        printf("\n");
                }

                next_position[i] = start_position[i] +
                                   k[i]*((node_counter*tk*v_p[i]) -
                                         (node_counter*node_counter*tk*tk*a_r[i]/2));
            }
        }
        else if(node_counter<=jedn[i])
        { // drugi etap - ruch jednostajny
            if(debug)
            {
                printf("%.16f ", (start_position[i] +
                                  k[i]*(s_przysp[i] +
                                        (node_counter*tk - fabs(v_r[i]-v_p[i])/a_r[i])*v_r[i])));
                if(i==7)
                    printf("\n");
            }
            next_position[i] = start_position[i] +
                               k[i]*(s_przysp[i] + (node_counter*tk - fabs(v_r[i]-v_p[i])/a_r[i])*v_r[i]);
        }
        else if(node_counter<=td.interpolation_node_no)
        { //trzeci etap
            if(v_k[i]<=v_r[i])
            { //hamowanie w trzecim etapie
                if(debug)
                {
                    printf("%.16f ", (start_position[i] +
                                      k[i]*(s_przysp[i] + s_jedn[i] +
                                            (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*v_r[i] -
                                            (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*(node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*a_r[i]/2)));
                    if(i==7)
                        printf("\n");
                }
                next_position[i] = start_position[i] +
                                   k[i]*(s_przysp[i] + s_jedn[i] +
                                         (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*v_r[i] -
                                         (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*(node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*a_r[i]/2);
            }
            else
            { //przyspieszanie w trzecim etapie
                if(debug)
                {
                    printf("%.16f ", (start_position[i] +
                                      k[i]*(s_przysp[i] + s_jedn[i] +
                                            (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*v_r[i] +
                                            (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*(node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*a_r[i]/2)));
                    if(i==7)
                        printf("\n");
                }
                next_position[i] = start_position[i] +
                                   k[i]*(s_przysp[i] + s_jedn[i] +
                                         (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*v_r[i] +
                                         (node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*(node_counter*tk + fabs(v_r[i]-v_k[i])/a_r[i] - t_max)*a_r[i]/2);
            }
        }
    }

}

bool ecp_smooth_generator::load_file_with_path (char* file_name)
{
    // Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,

    char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
    POSE_SPECIFICATION ps;     // Rodzaj wspolrzednych
    uint64_t e;       // Kod bledu systemowego
    uint64_t number_of_poses; // Liczba zapamietanych pozycji
    uint64_t i, j;    // Liczniki petli
    bool first_time = true; // Znacznik
    int extra_info;
    double vp[MAX_SERVOS_NR];
    double vk[MAX_SERVOS_NR];
    double v[MAX_SERVOS_NR];
    double a[MAX_SERVOS_NR];	// Wczytane wspolrzedne
    double coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne

    ifstream from_file(file_name); // otworz plik do odczytu
    if (!from_file)
    {
        perror(file_name);
        throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    if ( !(from_file >> coordinate_type) )
    {
        from_file.close();
        throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
    }

    // Usuwanie spacji i tabulacji
    i = 0;
    j = 0;
    while ( coordinate_type[i] == ' ' || coordinate_type[i] == '\t')
        i++;
    while ( coordinate_type[i] != ' '   && coordinate_type[i] != '\t' &&
            coordinate_type[i] != '\n'  && coordinate_type[i] != '\r' &&
            coordinate_type[j] != '\0' )
    {
        coordinate_type[j] = toupper(coordinate_type[i]);
        i++;
        j++;
    }
    coordinate_type[j] = '\0';

    if ( !strcmp(coordinate_type, "MOTOR") )
    {
        ps = MOTOR;
    }
    else if ( !strcmp(coordinate_type, "JOINT") )
    {
        ps = JOINT;
    }
    else if ( !strcmp(coordinate_type, "XYZ_ANGLE_AXIS") )
    {
        ps = XYZ_ANGLE_AXIS;
    }
    else if ( !strcmp(coordinate_type, "XYZ_EULER_ZYZ") )
    {
        ps = XYZ_EULER_ZYZ;
    }
    //	else if ( !strcmp(coordinate_type, "POSE_FORCE_LINEAR") )
    //		ps = POSE_FORCE_LINEAR;
    else
    {
        from_file.close();
        throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
    }
    // printf("po coord type %d\n", ps);
    if ( !(from_file >> number_of_poses) )
    {
        from_file.close();
        throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
    }
    // printf("po number of poses %d\n", number_of_poses);
    flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
    // printf("po flush pose list\n");
    for ( i = 0; i < number_of_poses; i++)
    {
        // printf("w petli\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> vp[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                from_file.close();
                throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po vp\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> vk[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                from_file.close();
                throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po vk\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> v[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                from_file.close();
                throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po v\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> a[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                from_file.close();
                throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po a\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> coordinates[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                from_file.close();
                throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po coord\n");

        /*		if (ps == POSE_FORCE_LINEAR)
        		{ 
        			if ( !(from_file >> extra_info) ) 
        			{ // Zabezpieczenie przed danymi nienumerycznymi
        				from_file.close();
        				throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        			}
        			if (first_time) 
        			{
        				// Tworzymy glowe listy
        				first_time = false;
        				create_pose_list_head(ps, vp, vk, v, coordinates);
        			} 
        			else 
        			{
        				// Wstaw do listy nowa pozycje
        				insert_pose_list_element(ps, vp, vk, v, coordinates);
        			}
        		} 
        		else 
        		{*/
        if (first_time)
        {
            // Tworzymy glowe listy
            first_time = false;
            create_pose_list_head(ps, vp, vk, v, a, coordinates);
            // printf("Pose list head: %d, %f, %f, %f, %f\n", ps, vp[0], vk[0], v[0], a[0]);
        }
        else
        {
            // Wstaw do listy nowa pozycje
            insert_pose_list_element(ps, vp, vk, v, a, coordinates);
            // printf("Pose list element: %d, %f, %f, %f, %f\n", ps, vp[0], vk[0], v[0], a[0]);
        }
        //		}
    } // end: for
    from_file.close();
    return true;
}
; // end: load_file()

void ecp_smooth_generator::calculate(void)
{
    double s[MAX_SERVOS_NR];
    double t;
    double v_1, v_2;
    int i, tmp;
    double tk=10*STEP; //czas jednego makrokroku

    for(i=0; i<MAX_SERVOS_NR; i++)
    {
        // Obliczenie drog dla wszystkich osi
        s[i]=fabs(final_position[i]-start_position[i]);
        // kierunek ruchu
        if(final_position[i]-start_position[i] < 0)
        {
            k[i]=-1;
        }
        else
        {
            k[i]=1;
        }

        if(s[i]==0)
        {
            t=0;
        }
        else if(
            ((v_r[i]>=v_p[i]) && (v[i]>=v_k[i]) && (((2*v_p[i]*(v_r[i]-v_p[i]) + 2*v_r[i]*(v_r[i]-v_k[i])
                                                    +((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) - ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]<v_p[i]) && (v[i]>=v_k[i]) && (((2*v_p[i]*(v_p[i]-v_r[i]) + 2*v_r[i]*(v_r[i]-v_k[i])
                                                    -((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) - ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]>=v_p[i]) && (v[i]<v_k[i]) && (((2*v_p[i]*(v_r[i]-v_p[i]) + 2*v_r[i]*(v_k[i] - v_r[i])
                                                    +((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) + ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
            ||
            ((v_r[i]<v_p[i]) && (v[i]<v_k[i]) && (((2*v_p[i]*(v_p[i]-v_r[i]) + 2*v_r[i]*(v_k[i] - v_r[i])
                                                    -((v_r[i]-v_p[i])*(v_r[i]-v_p[i])) + ((v_k[i] - v_r[i])*(v_k[i] - v_r[i])))/(2*a_r[i])) < s[i]))
        )

            // zwykly ruch w 3 etapach
        {
            if(debug)
            {
                printf("%d - 3 etapy\n", i);
            }
            // Obliczenie najdluzszego czasu
            if((v_r[i]>=v_p[i]) && (v_r[i]>=v_k[i]))
            {// pierwszy model ruchu - przyspieszanie / hamowanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) - (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) + (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]>=v_p[i]) && (v_r[i]<v_k[i]))
            {// drugi model ruchu - przyspieszanie / przyspieszanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) - (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) - (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]<v_p[i]) && (v_r[i]>=v_k[i]))
            {// trzeci model ruchu - hamowanie / hamowanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) + (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) + (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }
            else if((v_r[i]<v_p[i]) && (v_r[i]<v_k[i]))
            {// czwarty model ruchu - hamowanie / przyspieszanie
                t =
                    fabs((v_r[i]-v_p[i])/a_r[i]) +
                    ((2*s[i]*a_r[i] -2*v_p[i]*fabs(v_r[i] - v_p[i]) + (v_r[i]-v_p[i])*(v_r[i]-v_p[i]) - 2*v_r[i]*fabs(v_k[i]-v_r[i]) - (v_k[i] - v_r[i])*(v_k[i] - v_r[i])) /
                     (2*a_r[i]*v_r[i])) +
                    fabs((v_r[i]-v_k[i])/a_r[i]);
            }

        } //end - obliczanie czasu w 3 etapach
        else //jesli droga jest krotsza niz osiagnieta przy przyspieszaniu i hamowaniu przy zadanych a i v
        {
            switch(i)
            {
            case 0:
                sr_ecp_msg.message("Redukcja predkosci w osi 0");
                break;
            case 1:
                sr_ecp_msg.message("Redukcja predkosci w osi 1");
                break;
            case 2:
                sr_ecp_msg.message("Redukcja predkosci w osi 2");
                break;
            case 3:
                sr_ecp_msg.message("Redukcja predkosci w osi 3");
                break;
            case 4:
                sr_ecp_msg.message("Redukcja predkosci w osi 4");
                break;
            case 5:
                sr_ecp_msg.message("Redukcja predkosci w osi 5");
                break;
            case 6:
                sr_ecp_msg.message("Redukcja predkosci w osi 6");
                break;
            case 7:
                sr_ecp_msg.message("Redukcja predkosci w osi 7");
                break;
            }
            if(debug)
            {
                printf("%d - 2 etapy\n", i);
            }
            v_1=sqrt(v_p[i]*v_p[i]/2 + v_k[i]*v_k[i]/2 + s[i]*a_r[i]);
            if(v_1>=v_p[i] && v_1>=v_k[i])
            {
                v_r[i]=v_1;
            }
            else
            {
                v_1=(v_p[i] + v_k[i])/2;
                if((v_1<v_p[i] && v_1>=v_k[i]) || (v_1>=v_p[i] && v_1<v_k[i]))
                    v_r[i]=v_1;
                else
                {
                    v_1=sqrt(v_p[i]*v_p[i]/2 + v_k[i]*v_k[i]/2 - s[i]*a_r[i]);
                    if(v_1<v_p[i] && v_1<v_k[i])
                    {
                        v_r[i] = v_1;
                    }
                    else
                    {
                        printf("Blad w obliczaniu predkosci w 2 etapach!!\n");
                        throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
                    }

                }
            }

            t = (fabs(v_r[i] - v_p[i]) + fabs(v_r[i] - v_k[i]))/a_r[i];
        }
        if(t>t_max)
            t_max=t;
    }

    //kwantyzacja czasu
    if(ceil(t_max/tk)*tk != t_max)
    {
        t_max = ceil(t_max/tk);
        t_max = t_max*tk;
    }

    // Obliczenie predkosci dla wszystkich osi
    for(i=0; i<MAX_SERVOS_NR; i++)
        if(s[i]==0)
            v_r[i]=0;
        else
        {//pierszy model ruchu - przyspieszanie / hamowanie
            v_1=(v_p[i]+v_k[i]+a_r[i]*t_max)/2 + sqrt(-4*v_p[i]*v_p[i]
                    -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]+8*v_p[i]*a_r[i]*t_max+8*v_k[i]
                    *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                    -16*a_r[i]*s[i])/4;

            v_2=(v_p[i]+v_k[i]+a_r[i]*t_max)/2 - sqrt(-4*v_p[i]*v_p[i]
                    -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]+8*v_p[i]*a_r[i]*t_max+8*v_k[i]
                    *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                    -16*a_r[i]*s[i])/4;

            if((v_1>=0)&&(v_1<=v_r[i])&&(v_1>=v_p[i])&&(v_1>=v_k[i]))
                v_r[i]=v_1;
            else if((v_2>=0)&&(v_2<=v_r[i])&&(v_2>=v_p[i])&&(v_2>=v_k[i]))
                v_r[i]=v_2;
            else
            {//drugi model ruchu - przyspieszanie / przyspieszanie
                v_1 = (v_k[i]*v_k[i] - 2*a_r[i]*s[i] - v_p[i]*v_p[i])/(2*(v_k[i]-v_p[i]-a_r[i]*t_max));

                if((v_1>=0)&&(v_1<=v_r[i])&&(v_1>=v_p[i])&&(v_1<v_k[i]))
                    v_r[i]=v_1;
                else
                {//trzeci model ruchu - hamowanie / hamowanie
                    v_1 = (-2*a_r[i]*s[i] + v_p[i]*v_p[i] - v_k[i]*v_k[i])/(2*(v_p[i]-v_k[i]-a_r[i]*t_max));

                    if((v_1>=0)&&(v_1<=v_r[i])&&(v_1<v_p[i])&&(v_1>=v_k[i]))
                        v_r[i]=v_1;
                    else
                    {//czwarty model ruchu - hamowanie / przyspieszanie
                        v_1=(v_p[i]+v_k[i]-a_r[i]*t_max)/2 + sqrt(-4*v_p[i]*v_p[i]
                                -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]-8*v_p[i]*a_r[i]*t_max-8*v_k[i]
                                *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                                +16*a_r[i]*s[i])/4;

                        v_2=(v_p[i]+v_k[i]-a_r[i]*t_max)/2 - sqrt(-4*v_p[i]*v_p[i]
                                -4*v_k[i]*v_k[i]+8*v_p[i]*v_k[i]-8*v_p[i]*a_r[i]*t_max-8*v_k[i]
                                *a_r[i]*t_max+4*a_r[i]*a_r[i]*t_max*t_max
                                +16*a_r[i]*s[i])/4;

                        if((v_1>=0)&&(v_1<=v_r[i])&&(v_1<v_p[i])&&(v_1<v_k[i]))
                            v_r[i]=v_1;
                        else if((v_2>=0)&&(v_2<=v_r[i])&&(v_2<v_p[i])&&(v_2<v_k[i]))
                            v_r[i]=v_2;
                        else
                        {//blad - brak rozwiazania
                            printf("blad! nie da sie obliczyc predkosci (%d)\n", i);
                            throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);

                            v_r[i]=0;
                        }
                    }
                }
            }

            if(s[i]==0)
                v_r[i]=0;

        }

    // Wypelnienie struktury td
    td.interpolation_node_no = (int)round(t_max / tk);
    td.internode_step_no = 10;
    td.value_in_step_no = td.internode_step_no - 2;

    for(i=0;i<MAX_SERVOS_NR;i++)
        td.coordinate_delta[i] = final_position[i]-
                                 start_position[i];
    if(debug)
    {
        printf("makrokroki: %d, mikrokroki: %d, czas kroku: %f, step: %f\n", td.interpolation_node_no, td.internode_step_no, tk, STEP);
        printf("t: %f\n", t_max);
        printf("v: %f, %f, %f, %f, %f, %f, %f, %f\n", v_r[0], v_r[1], v_r[2], v_r[3], v_r[4], v_r[5], v_r[6], v_r[7]);
    }

    for(i=0;i<MAX_SERVOS_NR;i++)
    {
        przysp[i]=fabs((v_r[i]-v_p[i])/(a_r[i]*tk));
        jedn[i]=(t_max-(fabs(v_r[i]-v_k[i])/a_r[i]))/tk;

        if(v_r[i]>=v_p[i])
        {
            s_przysp[i]=(2*v_p[i]*(v_r[i]-v_p[i]) + (v_r[i] - v_p[i])*(v_r[i] - v_p[i]))/(2*a_r[i]);
        }
        else
        {
            s_przysp[i]=(2*v_p[i]*(v_p[i] - v_r[i]) - (v_r[i] - v_p[i])*(v_r[i] - v_p[i]))/(2*a_r[i]);
        }

        s_jedn[i]= (t_max - (fabs(v_r[i] - v_p[i]) + fabs(v_r[i] - v_k[i]))/a_r[i])*v_r[i];
    }

    v_grip =	(final_position[i]/td.interpolation_node_no);
    if(v_grip<v_grip_min)
        v_grip=v_grip_min;


} //end - calculate

void ecp_smooth_generator::flush_pose_list ( void )
{
    pose_list.clear();
}
; // end: flush_pose_list
// -------------------------------------------------------
void ecp_smooth_generator::initiate_pose_list(void)
{
    pose_list_iterator = pose_list.begin();
};
// -------------------------------------------------------
void ecp_smooth_generator::next_pose_list_ptr (void)
{
    if (pose_list_iterator != pose_list.end())
        pose_list_iterator++;
}
// -------------------------------------------------------
void ecp_smooth_generator::get_pose (void)
{
    int i;

    td.arm_type = pose_list_iterator->arm_type;
    for(i=0; i<MAX_SERVOS_NR; i++)
    {
        v_p[i]=pose_list_iterator->v_p[i];
        v_k[i]=pose_list_iterator->v_k[i];
        v[i]=pose_list_iterator->v[i];
        a[i]=pose_list_iterator->a[i];
        final_position[i]=pose_list_iterator->coordinates[i];
    }

}
// -------------------------------------------------------
void ecp_smooth_generator::set_pose (POSE_SPECIFICATION ps, double vp[MAX_SERVOS_NR], double vk[MAX_SERVOS_NR], double vv[MAX_SERVOS_NR], double aa[MAX_SERVOS_NR], double c[MAX_SERVOS_NR])
{
    pose_list_iterator->arm_type = ps;
    memcpy(pose_list_iterator->coordinates, c, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->v_p, vp, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->v_k, vk, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->v, vv, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->a, aa, MAX_SERVOS_NR*sizeof(double));
}
// -------------------------------------------------------
bool ecp_smooth_generator::is_pose_list_element ( void )
{
    // sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
    if ( pose_list_iterator != pose_list.end())
    {
        return true;
    }
    else
    {
        return false;
    }
}
// -------------------------------------------------------
bool ecp_smooth_generator::is_last_list_element ( void )
{
    // sprawdza czy aktualnie wskazywany element listy ma nastepnik
    // jesli <> nulla
    if ( pose_list_iterator != pose_list.end() )
    {
        if ( (++pose_list_iterator) != pose_list.end() )
        {
            --pose_list_iterator;
            return false;
        }
        else
        {
            --pose_list_iterator;
            return true;
        }
        ; // end if
    }
    return false;
};
// -------------------------------------------------------

void ecp_smooth_generator::create_pose_list_head (POSE_SPECIFICATION ps, double v_p[MAX_SERVOS_NR], double v_k[MAX_SERVOS_NR], double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR])
{
    pose_list.push_back(ecp_smooth_taught_in_pose(ps, v_p, v_k, v, a, coordinates));
    pose_list_iterator = pose_list.begin();
}


void ecp_smooth_generator::insert_pose_list_element (POSE_SPECIFICATION ps, double v_p[MAX_SERVOS_NR], double v_k[MAX_SERVOS_NR], double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR])
{
    pose_list.push_back(ecp_smooth_taught_in_pose(ps, v_p, v_k, v, a, coordinates));
    pose_list_iterator++;
}

// -------------------------------------------------------
int ecp_smooth_generator::pose_list_length(void)
{
    return pose_list.size();
};

bool ecp_smooth_generator::load_a_v_min (char* file_name)
{
    uint64_t e;       // Kod bledu systemowego
    uint64_t j;    // Liczniki petli
    ifstream from_file(file_name); // otworz plik do odczytu

    if (!from_file)
    {
        // printf("error\n");
        perror(file_name);
        throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    if ( !(from_file >> v_grip_min) )
    { // Zabezpieczenie przed danymi nienumerycznymi
        from_file.close();
        throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
    }

    from_file.close();
    return true;
} // end: load_a_v_min()

bool ecp_smooth_generator::load_a_v_max (char* file_name)
{
    uint64_t e;       // Kod bledu systemowego
    uint64_t j;    // Liczniki petli
    ifstream from_file(file_name); // otworz plik do odczytu

    if (!from_file)
    {
        // printf("error\n");
        perror(file_name);
        throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_motor[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_motor[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_joint[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_joint[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_zyz[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_zyz[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_aa[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_aa[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            from_file.close();
            throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    from_file.close();
    return true;
} // end: load_a_v_max()

ecp_smooth_generator::ecp_smooth_generator (ecp_task& _ecp_task, bool _is_synchronised)
        :
        ecp_delta_generator (_ecp_task), debug(false)
{
    int i;
    double vp[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double vk[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double v[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double a[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};



    int size = 1 + strlen(ecp_t.mrrocpp_network_path) + strlen("data/a_v_max.txt");
    char * path1 = new char[size];
    // Stworzenie sciezki do pliku.
    strcpy(path1, ecp_t.mrrocpp_network_path);
    sprintf(path1, "%sdata/a_v_max.txt", ecp_t.mrrocpp_network_path);

    size = 1 + strlen(ecp_t.mrrocpp_network_path) + strlen("data/a_v_min.txt");
    char * path2 = new char[size];
    // Stworzenie sciezki do pliku.
    strcpy(path2, ecp_t.mrrocpp_network_path);
    sprintf(path2, "%sdata/a_v_min.txt", ecp_t.mrrocpp_network_path);

    load_a_v_max(path1);
    load_a_v_min(path2);

    delete[] path1;
    delete[] path2;

    is_synchronised = _is_synchronised;
    //v_grip_min=5;
}
; // end : konstruktor

ecp_smooth_generator::ecp_smooth_generator (ecp_task& _ecp_task, bool _is_synchronised, bool _debug)
        :
        ecp_delta_generator (_ecp_task)
{
    int i;
    double vp[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double vk[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double v[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double a[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};



    int size = 1 + strlen(ecp_t.mrrocpp_network_path) + strlen("data/a_v_max.txt");
    char * path1 = new char[size];
    // Stworzenie sciezki do pliku.
    strcpy(path1, ecp_t.mrrocpp_network_path);
    sprintf(path1, "%sdata/a_v_max.txt", ecp_t.mrrocpp_network_path);

    size = 1 + strlen(ecp_t.mrrocpp_network_path) + strlen("data/a_v_min.txt");
    char * path2 = new char[size];
    // Stworzenie sciezki do pliku.
    strcpy(path2, ecp_t.mrrocpp_network_path);
    sprintf(path2, "%sdata/a_v_min.txt", ecp_t.mrrocpp_network_path);

    load_a_v_max(path1);
    load_a_v_min(path2);

    delete[] path1;
    delete[] path2;

    is_synchronised = _is_synchronised;
    debug = _debug;
    //v_grip_min=5;
}
; // end : konstruktor

bool ecp_smooth_generator::first_step ()
{

    initiate_pose_list();
    get_pose();


    first_interval=true;
    switch ( td.arm_type )
    {

    case MOTOR:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = MOTOR;
        the_robot->EDP_data.get_arm_type = MOTOR;
        the_robot->EDP_data.motion_type = ABSOLUTE;
        break;
    case JOINT:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = JOINT;
        the_robot->EDP_data.get_arm_type = JOINT;
        the_robot->EDP_data.motion_type = ABSOLUTE;
        break;
    case XYZ_EULER_ZYZ:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
        the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
        the_robot->EDP_data.motion_type = ABSOLUTE;
        break;
    case XYZ_ANGLE_AXIS:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
        the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
        the_robot->EDP_data.motion_type = ABSOLUTE;
        break;
    default:
        throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    } // end : switch ( td.arm_type )



    return true;

}
; // end: bool ecp_smooth_generator::first_step ( )

bool ecp_smooth_generator::next_step ()
{
    int i;
    double tk=10*STEP; //czas jednego makrokroku


    // ---------------------------------   FIRST INTERVAL    ---------------------------------------
    if ( first_interval )
    {
        // printf("first interval\n");

        t_max=0;
        get_pose();


        // Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
        // aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
        // to dopiero execute_motion po wyjsciu z first_step.

        switch ( td.arm_type )
        {
        case MOTOR:
            for(i=0;i<MAX_SERVOS_NR;i++)
                start_position[i]=the_robot->EDP_data.current_motor_arm_coordinates[i];
            for(i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_motor[i]*v[i];
                a_r[i]=a_max_motor[i]*a[i];
                v_p[i]=v_max_motor[i]*v_p[i];
                v_k[i]=v_max_motor[i]*v_k[i];
            }
            calculate();
            break;

        case JOINT:
            for(i=0;i<MAX_SERVOS_NR;i++)
                start_position[i]=the_robot->EDP_data.current_joint_arm_coordinates[i];
            for(i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_joint[i]*v[i];
                a_r[i]=a_max_joint[i]*a[i];
                v_p[i]=v_max_joint[i]*v_p[i];
                v_k[i]=v_max_joint[i]*v_k[i];
            }
            calculate();
            break;

        case XYZ_EULER_ZYZ:
            for(i=0;i<6;i++)
                start_position[i]=the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
            start_position[6]=the_robot->EDP_data.current_gripper_coordinate;
            start_position[7]=0.0;
            for(i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_zyz[i]*v[i];
                a_r[i]=a_max_zyz[i]*a[i];
                v_p[i]=v_max_zyz[i]*v_p[i];
                v_k[i]=v_max_zyz[i]*v_k[i];
            }
            calculate();
            break;
        case XYZ_ANGLE_AXIS:
            for(i=0;i<6;i++)
                start_position[i]=the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
            start_position[6]=the_robot->EDP_data.current_gripper_coordinate;
            start_position[7]=0.0;
            for(i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_aa[i]*v[i];
                a_r[i]=a_max_aa[i]*a[i];
                v_p[i]=v_max_aa[i]*v_p[i];
                v_k[i]=v_max_aa[i]*v_k[i];
            }
            calculate();
            break;
        default:
            throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
        } // end:switch

        first_interval = false;
        node_counter=0;

    }	// end:if FIRST INTERVAL
    // -------------------------------------------------------------------------------------------


    // Kontakt z MP
    if (node_counter == td.interpolation_node_no)
    { // Koniec odcinka
        if(is_last_list_element())	//ostatni punkt
        {
            ecp_t.ecp_termination_notice();
            return false;
        }
        else
        {
            t_max=0;
            for(i=0; i<MAX_SERVOS_NR; i++)
                start_position[i]=pose_list_iterator->coordinates[i];

            next_pose_list_ptr();

            if(debug)
            {
                printf("nastepny punkt\n");
            }

            get_pose();

            // Przepisanie danych z EDP_MASTER do obrazu robota
            the_robot->get_reply();

            switch ( td.arm_type )
            {
            case MOTOR:
                for(i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_motor[i]*v[i];
                    a_r[i]=a_max_motor[i]*a[i];
                    v_p[i]=v_max_motor[i]*v_p[i];
                    v_k[i]=v_max_motor[i]*v_k[i];
                }
                calculate();
                break;

            case JOINT:
                for(i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_joint[i]*v[i];
                    a_r[i]=a_max_joint[i]*a[i];
                    v_p[i]=v_max_joint[i]*v_p[i];
                    v_k[i]=v_max_joint[i]*v_k[i];
                }
                calculate();
                break;

            case XYZ_EULER_ZYZ:
                for(i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_zyz[i]*v[i];
                    a_r[i]=a_max_zyz[i]*a[i];
                    v_p[i]=v_max_zyz[i]*v_p[i];
                    v_k[i]=v_max_zyz[i]*v_k[i];
                }
                calculate();
                break;
            case XYZ_ANGLE_AXIS:
                for(i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_aa[i]*v[i];
                    a_r[i]=a_max_aa[i]*a[i];
                    v_p[i]=v_max_aa[i]*v_p[i];
                    v_k[i]=v_max_aa[i]*v_k[i];
                }
                calculate();
                break;
            default:
                throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
            } // end:switch

            node_counter=0;

            ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
            ecp_t.mp_buffer_receive_and_send ();
        }
    } //koniec: nastepny punkt trajektorii
    else
    { // w trakcie interpolacji
        ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
        ecp_t.mp_buffer_receive_and_send ();
    }

    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.get_type = NOTHING_DV;
    the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;


    switch ( td.arm_type )
    {
    case MOTOR:
        the_robot->EDP_data.instruction_type = SET;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = MOTOR;
        the_robot->EDP_data.motion_type = ABSOLUTE;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_motor_arm_coordinates[i] = next_position[i];

            //PROBA Z CHWYTAKIEM

            if(the_robot->robot_name == ROBOT_IRP6_ON_TRACK)
                i=8;
            else if(the_robot->robot_name == ROBOT_IRP6_POSTUMENT)
                i=7;

            if(v_grip*node_counter < final_position[i])
            {
                the_robot->EDP_data.next_motor_arm_coordinates[i] = v_grip*node_counter;
            }
            else
            {
                the_robot->EDP_data.next_motor_arm_coordinates[i] = final_position[i];
            }
        }
        else
        {
            //OSTATNI PUNKT
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_motor_arm_coordinates[i] = final_position[i];
        }
        break;

    case JOINT:
            the_robot->EDP_data.instruction_type = SET;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = JOINT;
        the_robot->EDP_data.motion_type = ABSOLUTE;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_joint_arm_coordinates[i] = next_position[i];

            //PROBA Z CHWYTAKIEM

            if(the_robot->robot_name == ROBOT_IRP6_ON_TRACK)
                i=8;
            else if(the_robot->robot_name == ROBOT_IRP6_POSTUMENT)
                i=7;

            if(v_grip*node_counter < final_position[i])
            {
                the_robot->EDP_data.next_joint_arm_coordinates[i] = v_grip*node_counter;
            }
            else
            {
                the_robot->EDP_data.next_joint_arm_coordinates[i] = final_position[i];
            }
        }
        else
        {
            //OSTATNI PUNKT
            generate_next_coords();
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_joint_arm_coordinates[i] = final_position[i];
        }
        break;

    case XYZ_EULER_ZYZ:
            the_robot->EDP_data.instruction_type = SET;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
        the_robot->EDP_data.motion_type = ABSOLUTE;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = next_position[i];

            //PROBA Z CHWYTAKIEM

            if(v_grip*node_counter < final_position[6])
            {
                the_robot->EDP_data.next_gripper_coordinate = v_grip*node_counter;
            }
            else
            {
                the_robot->EDP_data.next_gripper_coordinate = final_position[6];
            }
        }
        else
        {
            //OSTATNI PUNKT
            for (i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = final_position[i];
            the_robot->EDP_data.next_gripper_coordinate = final_position[6];
        }

        break;

    case XYZ_ANGLE_AXIS:
            the_robot->EDP_data.instruction_type = SET;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
        the_robot->EDP_data.motion_type = ABSOLUTE;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = next_position[i];

            the_robot->EDP_data.next_gripper_coordinate = next_position[6];
        }
        else
        {
            //OSTATNI PUNKT
            for (i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = final_position[i];
            the_robot->EDP_data.next_gripper_coordinate = final_position[6];
        }
        break;
    default:
            throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    }// end:switch


    return true;

} // end: BOOLEAN ecp_smooth_generator::next_step ( )

/**************************************************************************/
/**************/
/**************************************************************************/

bool ecp_tool_change_generator::first_step ()
{


    Homog_matrix tool_frame(tool_parameters[0], tool_parameters[1], tool_parameters[2]);
    tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame_m);

    the_robot->EDP_data.instruction_type = SET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = RMODEL_DV;
    the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
    the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = ABSOLUTE;


    return true;

}
; // end: bool ecp_smooth_pouring_generator::first_step ( )

bool ecp_tool_change_generator::next_step ()
{
    if (first_interval)
    {
        ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
    }
    else
    {
        ecp_t.ecp_termination_notice();
        return false;
    }




    first_interval=false;

    return true;

} // end: BOOLEAN ecp_smooth_pouring_generator::next_step ( )

void ecp_tool_change_generator::set_tool_parameters(double x, double y, double z)
{
    tool_parameters[0]=x;
    tool_parameters[1]=y;
    tool_parameters[2]=z;
}

ecp_tool_change_generator::ecp_tool_change_generator (ecp_task& _ecp_task, bool _is_synchronised)
        :ecp_smooth_generator (_ecp_task, _is_synchronised)
{

    set_tool_parameters(-0.18, 0.0, 0.25);

}
ecp_tool_change_generator::ecp_tool_change_generator (ecp_task& _ecp_task, bool _is_synchronised, bool _debug)
        :ecp_smooth_generator (_ecp_task, _is_synchronised, _debug)
{

    set_tool_parameters(-0.18, 0.0, 0.25);

}
