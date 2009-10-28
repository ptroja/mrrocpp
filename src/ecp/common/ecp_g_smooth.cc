// -------------------------------------------------------------------------
//                            ecp_g_smooth.cc
//            Effector Control Process (lib::ECP) - smooth generator
// Generator nie wykorzystujacy informacji o czasie ruchu
// autor: Przemek Pilacinski
// Ostatnia modyfikacja: 2008
// -------------------------------------------------------------------------


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth.h"
#include <fstream>
#include <string.h>
#include "ecp_mp/smooth_trajectory_pose.h"

#include "lib/datastr.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

void smooth::set_relative(void){
	type=2;
}

void smooth::set_absolute(void){
	type=1;
}

bool smooth::eq(double a, double b)
{
	const double EPS = 0.0001f;
	const double diff = a - b;
	return fabs((double) (diff < EPS));
}

void smooth::generate_next_coords (void)
{
    //funkcja obliczajaca polozenie w danym makrokroku
    const double tk=10*STEP;

    for(int i=0; i<MAX_SERVOS_NR; i++)
    {
        if(node_counter<przysp[i])
        { //pierwszy etap
			if (debug && i == 0) printf("etap 1: ");

            if(v_p[i]<=v_r[i])
            { //przyspieszanie w pierwszym etapie
                if(debug)
                {
                    printf("P%.8f ", ( start_position[i] + k[i]*(node_counter*v_p[i]*tk +
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
                    printf("H%.8f ", (start_position[i] +
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
			if (debug && i == 0) printf("etap 2: ");
            if(debug)
            {
                printf("J%.8f ", (start_position[i] +
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
			if (debug && i == 0) printf("etap 3: ");
            if(v_k[i]<=v_r[i])
            { //hamowanie w trzecim etapie
                if(debug)
                {
                    printf("H%.8f ", (start_position[i] +
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
                    printf("P%.8f ", (start_position[i] +
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

void smooth::set_trajectory(ecp_mp::common::Trajectory &trajectory)
{
	trajectory.showTime();

	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
	pose_list = &trajectory.getPoses();
	pose_list_iterator = pose_list->end();

	/*
	for(std::list<Trajectory::Pose>::iterator it = trajectory.getPoses()->begin(); it != trajectory.getPoses()->end(); ++it)
	{
         insert_pose_list_element(trajectory.getPoseSpecification(), (*it).startVelocity, (*it).endVelocity, (*it).velocity, (*it).accelerations, (*it).coordinates);
	}
	*/
}

void smooth::set_pose_from_xml(xmlNode *stateNode)
{
	uint64_t number_of_poses; // Liczba zapamietanych pozycji
	lib::POSE_SPECIFICATION ps;     // Rodzaj wspolrzednych
	double vp[MAX_SERVOS_NR];
	double vk[MAX_SERVOS_NR];
	double v[MAX_SERVOS_NR];
	double a[MAX_SERVOS_NR];	// Wczytane wspolrzedne
	double coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne

	xmlNode *cchild_node, *ccchild_node;
	xmlChar *coordinateType, *numOfPoses;
	xmlChar *xmlDataLine;

	coordinateType = xmlGetProp(stateNode, (const xmlChar *)"coordinateType");
	ps = lib::returnProperPS((char *)coordinateType);
	numOfPoses = xmlGetProp(stateNode, (const xmlChar *)"numOfPoses");
	number_of_poses = (uint64_t)atoi((const char *)numOfPoses);
	for(cchild_node = stateNode->children; cchild_node!=NULL; cchild_node = cchild_node->next)
	{
		if ( cchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cchild_node->name, (const xmlChar *)"Pose") )							{
			for(ccchild_node = cchild_node->children; ccchild_node!=NULL; ccchild_node = ccchild_node->next)
			{
				if ( ccchild_node->type == XML_ELEMENT_NODE ) {
					if(!xmlStrcmp(ccchild_node->name, (const xmlChar *)"StartVelocity"))
					{
						xmlDataLine = xmlNodeGetContent(ccchild_node);
						lib::setValuesInArray(vp, (const char *)xmlDataLine);
						xmlFree(xmlDataLine);
					}
					else if (!xmlStrcmp(ccchild_node->name, (const xmlChar *)"EndVelocity") )
					{
						xmlDataLine = xmlNodeGetContent(ccchild_node);
						lib::setValuesInArray(vk, (const char *)xmlDataLine);
						xmlFree(xmlDataLine);
					}
					else if (!xmlStrcmp(ccchild_node->name, (const xmlChar *)"Velocity") )
					{
						xmlDataLine = xmlNodeGetContent(ccchild_node);
						lib::setValuesInArray(v, (const char *)xmlDataLine);
						xmlFree(xmlDataLine);
					}
					else if (!xmlStrcmp(ccchild_node->name, (const xmlChar *)"Accelerations") )
					{
						xmlDataLine = xmlNodeGetContent(ccchild_node);
						lib::setValuesInArray(a, (const char *)xmlDataLine);
						xmlFree(xmlDataLine);
					}
					else if (!xmlStrcmp(ccchild_node->name, (const xmlChar *)"Coordinates") )
					{
						xmlDataLine = xmlNodeGetContent(ccchild_node);
						lib::setValuesInArray(coordinates, (const char *)xmlDataLine);
						xmlFree(xmlDataLine);
					}
				}
			}
			insert_pose_list_element(ps, vp, vk, v, a, coordinates);
		}
	}
	xmlFree(coordinateType);
	xmlFree(numOfPoses);
}

void smooth::load_trajectory_from_xml(const char* fileName, const char* nodeName)
{
	// Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,

	xmlDocPtr doc = xmlParseFile(fileName);
	xmlXIncludeProcess(doc);
	if(doc == NULL)
	{
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	xmlNodePtr root = xmlDocGetRootElement(doc);
	if(!root || !root->name)
	{
		xmlFreeDoc(doc);
		throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje

	for(xmlNodePtr cur_node = root->children; cur_node != NULL; cur_node = cur_node->next)
	{
		if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cur_node->name, (const xmlChar *) "SubTask" ) )
		{
			for(xmlNodePtr subTaskNode = cur_node->children; subTaskNode != NULL; subTaskNode = subTaskNode->next)
			{
				if ( subTaskNode->type == XML_ELEMENT_NODE  && !xmlStrcmp(subTaskNode->name, (const xmlChar *) "State" ) )
				{
					xmlChar *stateID = xmlGetProp(subTaskNode, (const xmlChar *) "id");
					if(stateID && !strcmp((const char *) stateID, nodeName))
					{
						for(xmlNodePtr child_node = subTaskNode->children; child_node != NULL; child_node = child_node->next)
						{
							if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"Trajectory") )
							{
								set_pose_from_xml(child_node);
							}
						}
					}
					xmlFree(stateID);
				}
			}
		}
		if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cur_node->name, (const xmlChar *) "State" ) )
		{
			xmlChar *stateID = xmlGetProp(cur_node, (const xmlChar *) "id");
			if(stateID && !strcmp((const char *)stateID, nodeName))
			{
				for(xmlNodePtr child_node = cur_node->children; child_node != NULL; child_node = child_node->next)
				{
					if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"Trajectory") )
					{
						set_pose_from_xml(child_node);
					}
				}
			}
			xmlFree(stateID);
		}
	}
	xmlFreeDoc(doc);
	xmlCleanupParser();
}

void smooth::load_file_with_path (const char* file_name)
{
    // Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,

	//printf("%s\n", file_name);
	//flushall();

    char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
    lib::POSE_SPECIFICATION ps;     // Rodzaj wspolrzednych
    uint64_t number_of_poses; // Liczba zapamietanych pozycji
    uint64_t i, j;    // Liczniki petli
    double vp[MAX_SERVOS_NR];
    double vk[MAX_SERVOS_NR];
    double v[MAX_SERVOS_NR];
    double a[MAX_SERVOS_NR];	// Wczytane wspolrzedne
    double coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne

    std::ifstream from_file(file_name); // otworz plik do odczytu
    if (!from_file.good())
    {
        perror(file_name);
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    if ( !(from_file >> coordinate_type) )
    {
        throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
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
        ps = lib::MOTOR;
    }
    else if ( !strcmp(coordinate_type, "JOINT") )
    {
        ps = lib::JOINT;
    }
    else if ( !strcmp(coordinate_type, "XYZ_ANGLE_AXIS") )
    {
        ps = lib::XYZ_ANGLE_AXIS;
    }
    else if ( !strcmp(coordinate_type, "XYZ_EULER_ZYZ") )
    {
        ps = lib::XYZ_EULER_ZYZ;
    }
    //	else if ( !strcmp(coordinate_type, "POSE_FORCE_LINEAR") )
    //		ps = POSE_FORCE_LINEAR;
    else
    {
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
    }
    // printf("po coord type %d\n", ps);
    if ( !(from_file >> number_of_poses) )
    {
        throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
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
                throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po vp\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> vk[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po vk\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> v[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po v\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> a[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po a\n");
        for ( j = 0; j < MAX_SERVOS_NR; j++)
        {
            if ( !(from_file >> coordinates[j]) )
            { // Zabezpieczenie przed danymi nienumerycznymi
                throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
            }
        }
        // printf("po coord\n");

        /*
        if (ps == POSE_FORCE_LINEAR)
        		{
        			if ( !(from_file >> extra_info) )
        			{ // Zabezpieczenie przed danymi nienumerycznymi
        				throw ecp_generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        			}
       				// Wstaw do listy nowa pozycje
       				insert_pose_list_element(ps, vp, vk, v, coordinates);
        		}
        		else
        		{
        */
            // Wstaw do listy nowa pozycje
            insert_pose_list_element(ps, vp, vk, v, a, coordinates);
            // printf("Pose list element: %d, %f, %f, %f, %f\n", ps, vp[0], vk[0], v[0], a[0]);
        //		}
    } // end: for
	// only for trajectory xml writing -> for now
	//Trajectory::writeTrajectoryToXmlFile(file_name, ps, *pose_list);
}

void smooth::reset(){
	flush_pose_list();
}


//wczytuje wspolrzedne punkt���w poprzez funkcje
//poki co przy zmiane trybu nalezy usunac instniejaca instancje smooth_generatora i stworzyc nowa.
void smooth::load_coordinates(lib::POSE_SPECIFICATION ps, double vp[MAX_SERVOS_NR], double vk[MAX_SERVOS_NR], double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR])
{
	// Wstaw do listy nowa pozycje
	insert_pose_list_element(ps, vp, vk, v, a, coordinates);
}

void smooth::load_coordinates(lib::POSE_SPECIFICATION ps, double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6, double cor7){

	double vp[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double vk[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double v[MAX_SERVOS_NR]={0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 10.0};
	double a[MAX_SERVOS_NR]={0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 5.0};
	double coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne

	coordinates[0]=cor0;
	coordinates[1]=cor1;
	coordinates[2]=cor2;
	coordinates[3]=cor3;
	coordinates[4]=cor4;
	coordinates[5]=cor5;
	coordinates[6]=cor6;
	coordinates[7]=cor7;

	// Wstaw do listy nowa pozycje
	insert_pose_list_element(ps, vp, vk, v, a, coordinates);
}

void smooth::calculate(void)
{
    double s[MAX_SERVOS_NR];
    double t;
    double v_1, v_2;
    double tk=10*STEP; //czas jednego makrokroku

    for(int i=0; i<MAX_SERVOS_NR; i++)
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

        if(eq(s[i], 0.0))
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
                        throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
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
    for(int i=0; i<MAX_SERVOS_NR; i++)
        if(eq(s[i], 0.0))
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
                            throw ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);

                            v_r[i]=0;
                        }
                    }
                }
            }

            if(eq(s[i], 0.0))
                v_r[i]=0;

        }

    // Wypelnienie struktury td
    td.interpolation_node_no = lround(t_max / tk);
    td.internode_step_no = 10;
    td.value_in_step_no = td.internode_step_no - 2;

    for(int i=0;i<MAX_SERVOS_NR;i++)
        td.coordinate_delta[i] = final_position[i]-
                                 start_position[i];
    if(debug)
    {
        printf("makrokroki: %d, mikrokroki: %d, czas kroku: %f, step: %f\n", td.interpolation_node_no, td.internode_step_no, tk, STEP);
        printf("t: %f\n", t_max);
        printf("v: %f, %f, %f, %f, %f, %f, %f, %f\n", v_r[0], v_r[1], v_r[2], v_r[3], v_r[4], v_r[5], v_r[6], v_r[7]);
    }

    for(int i=0;i<MAX_SERVOS_NR;i++)
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

    // TODO: fixit
//    v_grip = (final_position[i]/td.interpolation_node_no);
//    if(v_grip<v_grip_min)
        v_grip=v_grip_min;
}

void smooth::flush_pose_list ( void )
{
	pose_list->clear();
}

// -------------------------------------------------------return iterator to beginning of the list
void smooth::initiate_pose_list(void)
{
    pose_list_iterator = pose_list->begin();
}
// -------------------------------------------------------
void smooth::next_pose_list_ptr (void)
{
    if (pose_list_iterator != pose_list->end())
        pose_list_iterator++;
}

// -------------------------------------------------------get all previously saved elements from actual iterator
void smooth::get_pose (void)
{
	td.arm_type = pose_list_iterator->arm_type;
    for(int i=0; i<MAX_SERVOS_NR; i++)
    {
        v_p[i]=pose_list_iterator->v_p[i];
        v_k[i]=pose_list_iterator->v_k[i];
        v[i]=pose_list_iterator->v[i];
        a[i]=pose_list_iterator->a[i];
        final_position[i]=pose_list_iterator->coordinates[i];
    }
//	if(type==2)
//		for(i=0; i<MAX_SERVOS_NR; i++)
//			final_position[i]+=start_position[i];
}
// -------------------------------------------------------
void smooth::set_pose (lib::POSE_SPECIFICATION ps, double vp[MAX_SERVOS_NR], double vk[MAX_SERVOS_NR], double vv[MAX_SERVOS_NR], double aa[MAX_SERVOS_NR], double c[MAX_SERVOS_NR])
{
    pose_list_iterator->arm_type = ps;
    memcpy(pose_list_iterator->coordinates, c, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->v_p, vp, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->v_k, vk, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->v, vv, MAX_SERVOS_NR*sizeof(double));
    memcpy(pose_list_iterator->a, aa, MAX_SERVOS_NR*sizeof(double));
}
// -------------------------------------------------------
bool smooth::is_pose_list_element ( void )
{
    // sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
    if ( pose_list_iterator != pose_list->end())
    {
        return true;
    }
    else
    {
        return false;
    }
}
// -------------------------------------------------------
bool smooth::is_last_list_element ( void )
{
    // sprawdza czy aktualnie wskazywany element listy ma nastepnik
    // jesli <> nulla
    if ( pose_list_iterator != pose_list->end() )
    {
        if ( (++pose_list_iterator) != pose_list->end() )
        {
            --pose_list_iterator;
            return false;
        }
        else
        {
            --pose_list_iterator;
            return true;
        }
    }
    return false;
}
// -------------------------------------------------------

void smooth::insert_pose_list_element (lib::POSE_SPECIFICATION ps, double v_p[MAX_SERVOS_NR], double v_k[MAX_SERVOS_NR], double v[MAX_SERVOS_NR], double a[MAX_SERVOS_NR], double coordinates[MAX_SERVOS_NR])
{
    pose_list->push_back(ecp_mp::common::smooth_trajectory_pose(ps, coordinates, v, a, v_p, v_k));
    if(pose_list->size() == 1) {
    	pose_list_iterator = pose_list->begin();
    } else {
    	pose_list_iterator++;
    }
}

// -------------------------------------------------------
int smooth::pose_list_length(void)
{
    return pose_list->size();
}

void smooth::load_a_v_min (const char* file_name)
{
    std::ifstream from_file(file_name); // otworz plik do odczytu

    if (!from_file.good())
    {
        perror(file_name);
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    if ( !(from_file >> v_grip_min) )
    { // Zabezpieczenie przed danymi nienumerycznymi
        throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
    }
} // end: load_a_v_min()

void smooth::load_a_v_max (const char* file_name)
{
    uint64_t j;    // Liczniki petli
    std::ifstream from_file(file_name); // otworz plik do odczytu

    if (!from_file.good())
    {
        // printf("error\n");
        perror(file_name);
        throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_motor[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_motor[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_joint[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_joint[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_zyz[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_zyz[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }

    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> v_max_aa[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
    for ( j = 0; j < MAX_SERVOS_NR; j++)
    {
        if ( !(from_file >> a_max_aa[j]) )
        { // Zabezpieczenie przed danymi nienumerycznymi
            throw generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
        }
    }
} // end: load_a_v_max()

smooth::smooth (common::task::task& _ecp_task, bool _is_synchronised, bool _debug)
        :
        delta (_ecp_task), is_synchronised(_is_synchronised), debug(_debug)
{
	pose_list = new std::list<ecp_mp::common::smooth_trajectory_pose>();

	// Stworzenie sciezek do plików
	std::string path1(ecp_t.mrrocpp_network_path);
	path1 += "data/a_v_max.txt";

	std::string path2(ecp_t.mrrocpp_network_path);
	path2 += "data/a_v_min.txt";

	load_a_v_max(path1.c_str());
	load_a_v_min(path2.c_str());

	type=1;
	//v_grip_min=5;
}

//set necessary instructions, and other data for preparing the robot
bool smooth::first_step ()
{
    initiate_pose_list();
    get_pose();

    first_interval=true;
    switch ( td.arm_type )
    {

    case lib::MOTOR:
        the_robot->EDP_data.instruction_type = lib::GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = lib::MOTOR;
        the_robot->EDP_data.get_arm_type = lib::MOTOR;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        break;
    case lib::JOINT:
        the_robot->EDP_data.instruction_type = lib::GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = lib::JOINT;
        the_robot->EDP_data.get_arm_type = lib::JOINT;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        break;
    case lib::XYZ_EULER_ZYZ:
        the_robot->EDP_data.instruction_type = lib::GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
        the_robot->EDP_data.get_arm_type = lib::XYZ_EULER_ZYZ;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        break;
    case lib::XYZ_ANGLE_AXIS:
        the_robot->EDP_data.instruction_type = lib::GET;
        the_robot->EDP_data.get_type = ARM_DV;
        the_robot->EDP_data.set_type = ARM_DV;
        the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
        the_robot->EDP_data.get_arm_type = lib::XYZ_ANGLE_AXIS;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        break;
    default:
        throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    } // end : switch ( td.arm_type )

    return true;
}

bool smooth::next_step ()
{
    // ---------------------------------   FIRST INTERVAL    ---------------------------------------
    if ( first_interval )
    {
        t_max=0;
        get_pose();

        // Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
        // aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
        // to dopiero execute_motion po wyjsciu z first_step.

        switch ( td.arm_type )
        {
        case lib::MOTOR:
            for(int i=0;i<MAX_SERVOS_NR;i++){
                start_position[i]=the_robot->EDP_data.current_motor_arm_coordinates[i];
					 if(type==2)
						final_position[i]+=start_position[i];
				}
            for(int i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_motor[i]*v[i];
                a_r[i]=a_max_motor[i]*a[i];
                v_p[i]=v_max_motor[i]*v_p[i];
                v_k[i]=v_max_motor[i]*v_k[i];
            }
            calculate();
            break;

        case lib::JOINT:
            for(int i=0;i<MAX_SERVOS_NR;i++)
                start_position[i]=the_robot->EDP_data.current_joint_arm_coordinates[i];
            for(int i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_joint[i]*v[i];
                a_r[i]=a_max_joint[i]*a[i];
                v_p[i]=v_max_joint[i]*v_p[i];
                v_k[i]=v_max_joint[i]*v_k[i];
            }
            calculate();
            break;

        case lib::XYZ_EULER_ZYZ:
            for(int i=0;i<6;i++)
                start_position[i]=the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
            start_position[6]=the_robot->EDP_data.current_gripper_coordinate;
            start_position[7]=0.0;
            for(int i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_zyz[i]*v[i];
                a_r[i]=a_max_zyz[i]*a[i];
                v_p[i]=v_max_zyz[i]*v_p[i];
                v_k[i]=v_max_zyz[i]*v_k[i];
            }
            calculate();
            break;
        case lib::XYZ_ANGLE_AXIS:
            for(int i=0;i<6;i++)
                start_position[i]=the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
            start_position[6]=the_robot->EDP_data.current_gripper_coordinate;
            start_position[7]=0.0;
            for(int i=0; i<MAX_SERVOS_NR; i++)
            {
                v_r[i]=v_max_aa[i]*v[i];
                a_r[i]=a_max_aa[i]*a[i];
                v_p[i]=v_max_aa[i]*v_p[i];
                v_k[i]=v_max_aa[i]*v_k[i];
            }
            calculate();
            break;
        default:
            throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
        } // end:switch

        first_interval = false;
    }	// end:if FIRST INTERVAL
    // -------------------------------------------------------------------------------------------

    // Kontakt z MP
    if (node_counter-1 == td.interpolation_node_no)
    { // Koniec odcinka
        if(is_last_list_element())	//ostatni punkt
        {

            return false;
        }
        else
        {
            t_max=0;
            for(int i=0; i<MAX_SERVOS_NR; i++){
					if(type==1)
						start_position[i]=pose_list_iterator->coordinates[i];
					if(type==2)
						start_position[i]+=pose_list_iterator->coordinates[i];
				}
            next_pose_list_ptr();

            if(debug)
            {
                printf("nastepny punkt\n");
            }

            get_pose();

			if(type==2) {
				for(int i=0; i<MAX_SERVOS_NR; i++) {
					final_position[i]+=start_position[i];
				}
			}

            // Przepisanie danych z EDP_MASTER do obrazu robota
            switch ( td.arm_type )
            {
            case lib::MOTOR:
                for(int i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_motor[i]*v[i];
                    a_r[i]=a_max_motor[i]*a[i];
                    v_p[i]=v_max_motor[i]*v_p[i];
                    v_k[i]=v_max_motor[i]*v_k[i];
                }
                calculate();
                break;

            case lib::JOINT:
                for(int i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_joint[i]*v[i];
                    a_r[i]=a_max_joint[i]*a[i];
                    v_p[i]=v_max_joint[i]*v_p[i];
                    v_k[i]=v_max_joint[i]*v_k[i];
                }
                calculate();
                break;

            case lib::XYZ_EULER_ZYZ:
                for(int i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_zyz[i]*v[i];
                    a_r[i]=a_max_zyz[i]*a[i];
                    v_p[i]=v_max_zyz[i]*v_p[i];
                    v_k[i]=v_max_zyz[i]*v_k[i];
                }
                calculate();
                break;
            case lib::XYZ_ANGLE_AXIS:
                for(int i=0; i<MAX_SERVOS_NR; i++)
                {
                    v_r[i]=v_max_aa[i]*v[i];
                    a_r[i]=a_max_aa[i]*a[i];
                    v_p[i]=v_max_aa[i]*v_p[i];
                    v_k[i]=v_max_aa[i]*v_k[i];
                }
                calculate();
                break;
            default:
                throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
            } // end:switch
      		 node_counter=1;

        }
    } //koniec: nastepny punkt trajektorii


    // Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

    the_robot->EDP_data.instruction_type = lib::SET;
    the_robot->EDP_data.get_type = NOTHING_DV;
    the_robot->EDP_data.get_arm_type = lib::INVALID_END_EFFECTOR;


    switch ( td.arm_type )
    {
    case lib::MOTOR:
        the_robot->EDP_data.instruction_type = lib::SET;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.set_arm_type = lib::MOTOR;
        the_robot->EDP_data.motion_type = lib::ABSOLUTE;
        the_robot->EDP_data.next_interpolation_type = lib::MIM;
        the_robot->EDP_data.motion_steps = td.internode_step_no;
        the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            int i;
            for (i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_motor_arm_coordinates[i] = next_position[i];

            //PROBA Z CHWYTAKIEM

            if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
                i=8;
            } else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
                i=7;
            }

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
            for (int i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_motor_arm_coordinates[i] = final_position[i];
        }
        break;

    case lib::JOINT:
    	the_robot->EDP_data.instruction_type = lib::SET;
    	the_robot->EDP_data.set_type = ARM_DV; // ARM
    	the_robot->EDP_data.set_arm_type = lib::JOINT;
    	the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    	the_robot->EDP_data.next_interpolation_type = lib::MIM;
    	the_robot->EDP_data.motion_steps = td.internode_step_no;
    	the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            int i;
            for (i=0; i<MAX_SERVOS_NR; i++) {
                the_robot->EDP_data.next_joint_arm_coordinates[i] = next_position[i];
            }

            //PROBA Z CHWYTAKIEM

            if(the_robot->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
                i=8;
            } else if(the_robot->robot_name == lib::ROBOT_IRP6_POSTUMENT) {
                i=7;
            }

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
            for (int i=0; i<MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_joint_arm_coordinates[i] = final_position[i];
        }
        break;

    case lib::XYZ_EULER_ZYZ:
    	the_robot->EDP_data.instruction_type = lib::SET;
    	the_robot->EDP_data.set_type = ARM_DV; // ARM
    	the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
    	the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    	the_robot->EDP_data.next_interpolation_type = lib::MIM;
    	the_robot->EDP_data.motion_steps = td.internode_step_no;
    	the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (int i=0; i<6; i++)
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
            for (int i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = final_position[i];
            the_robot->EDP_data.next_gripper_coordinate = final_position[6];
        }
        break;

    case lib::XYZ_ANGLE_AXIS:
    	the_robot->EDP_data.instruction_type = lib::SET;
    	the_robot->EDP_data.set_type = ARM_DV; // ARM
    	the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
    	the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    	the_robot->EDP_data.next_interpolation_type = lib::MIM;
    	the_robot->EDP_data.motion_steps = td.internode_step_no;
    	the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

        if(node_counter < td.interpolation_node_no)
        {
            generate_next_coords();
            for (int i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = next_position[i];

            the_robot->EDP_data.next_gripper_coordinate = next_position[6];
        }
        else
        {
            //OSTATNI PUNKT
            for (int i=0; i<6; i++)
                the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = final_position[i];
            the_robot->EDP_data.next_gripper_coordinate = final_position[6];
        }
        break;
    default:
            throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    }// end:switch

    return true;
} // end: BOOLEAN ecp_smooth_generator::next_step ( )

/**************************************************************************/
/**************/
/**************************************************************************/

bool tool_change::first_step ()
{
    lib::Homog_matrix tool_frame(tool_parameters[0], tool_parameters[1], tool_parameters[2]);
    tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);

    the_robot->EDP_data.instruction_type = lib::SET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.set_type = RMODEL_DV;
    the_robot->EDP_data.set_rmodel_type = lib::TOOL_FRAME;
    the_robot->EDP_data.get_rmodel_type = lib::TOOL_FRAME;
    the_robot->EDP_data.set_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->EDP_data.get_arm_type = lib::XYZ_EULER_ZYZ;
    the_robot->EDP_data.motion_type = lib::ABSOLUTE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;

    return true;
}

bool tool_change::next_step ()
{
    return false;
}

void tool_change::set_tool_parameters(double x, double y, double z)
{
    tool_parameters[0]=x;
    tool_parameters[1]=y;
    tool_parameters[2]=z;
}

tool_change::tool_change (common::task::task& _ecp_task, bool _is_synchronised, bool _debug)
        : smooth (_ecp_task, _is_synchronised, _debug)
{
    set_tool_parameters(-0.18, 0.0, 0.25);
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
