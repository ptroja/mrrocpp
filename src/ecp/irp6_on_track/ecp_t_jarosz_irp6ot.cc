#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_jarosz.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp/irp6_on_track/ecp_t_jarosz_irp6ot.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


// KONSTRUKTORY
jarosz::jarosz(lib::configurator &_config) : base(_config)
{}

// methods for ECP template to redefine in concrete classes
void jarosz::task_initialization(void)
{
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);

    sr_ecp_msg->message("ECP loaded");
}


void jarosz::main_task_algorithm(void)
{
	// #####################################################################################
	// TESTOWANIE GENRATOROW o przyrost polozenia/orientacji
	// #####################################################################################

	lib::trajectory_description tdes;

	// -----------------------------------
	// 	tdes.arm_type = lib::XYZ_EULER_ZYZ;
	tdes.arm_type = lib::JOINT;
	// -----------------------------------

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //
	tdes.interpolation_node_no =200;
	tdes.internode_step_no = 10;
	tdes.value_in_step_no = tdes.internode_step_no -2;
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

	// --------------------------------------------------------------------------------------------
	/*
        	// Wspolrzedne wewn. JOINT  VER 1

        	tdes.coordinate_delta[0] = 0.0;								// tor jezdny d0
        	tdes.coordinate_delta[1] = 0.3;								// kolumna obrotowa
        	tdes.coordinate_delta[2] = 0.0;								// ramie dolne
        	tdes.coordinate_delta[3] = 0.0;								// ramie gorne
        	tdes.coordinate_delta[4] = 0.0;								// pochylenie kisci
        	tdes.coordinate_delta[5] = 0.0;								// obrot kisci
	 */
	// --------------------------------------------------------------------------------------------

	tdes.coordinate_delta[0] = 0.0;								// tor jezdny d0
	tdes.coordinate_delta[1] = 0.7853981633974483;		// kolumna obrotowa (45 stopni)
	tdes.coordinate_delta[2] = 0.0;								// ramie dolne
	tdes.coordinate_delta[3] = 0.0;								// ramie gorne
	tdes.coordinate_delta[4] = 0.0;								// pochylenie kisci
	tdes.coordinate_delta[5] = 0.0;								// obrot kisci

	// --------------------------------------------------------------------------------------------
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	/*
        	tdes.coordinate_delta[0] = 0.0;		// przyrost wspolrzednej X
        	tdes.coordinate_delta[1] = 0.0;		// przyrost wspolrzednej Y
        	tdes.coordinate_delta[2] = 0.2;		// przyrost wspolrzednej Z
        	tdes.coordinate_delta[3] = 0.0;		// przyrost wspolrzednej FI
        	tdes.coordinate_delta[4] = 0.0;		// przyrost wspolrzednej TETA
        	tdes.coordinate_delta[5] = 0.0;		// przyrost wspolrzednej PSI
	 */
	// --------------------------------------------------------------------------------------------

	// ####################################################################################################
	// Generator prostoliniowy o zadany przyrost polozenia/orientacji
	// ####################################################################################################
	/*
        	linear_generator lg(tdes);
         	sr_ecp_msg->message("Wykonywany jest ruch o zadany przyrost polozenia/orientacji");
        	Move (*irp6_on_track, NULL, lg);
         	sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// Interpolacja funckja liniowa z parabolicznymi odcinkami krzywoliniowymi,
	// czyli trapezoidalny profil predkosci
	// ####################################################################################################
	/*
        	double ta[]={0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
        	double tb[]={0.9, 0.9, 0.9, 0.9, 0.9, 0.9};
        	ecp_linear_parabolic_generator trapez(tdes, ta, tb);
         	sr_ecp_msg->message("Wykonywany jest ruch o zadany przyrost polozenia/orientacji");
        	Move (*irp6_on_track, NULL, trapez);
         	sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// Generator o zadany przyrost polozenia/orientacji wykorzystujacy do interpolacji wielomian 3 stopnia
	// ciaglosc predkosci
	// predkosc poczatkowa i koncowa moze byc zadawana
	// ####################################################################################################

	double vp[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double vk[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	common::generator::cubic cubic(*this, tdes, vp, vk);
	sr_ecp_msg->message("Wykonywany jest ruch o zadany przyrost polozenia/orientacji");
	cubic.Move();
	sr_ecp_msg->message("Ruch robota zakonczyl sie");

	// ####################################################################################################
	// Generator o zadany przyrost polozenia/orientacji wykorzystujacy do interpolacji wielomian 5 stopnia
	// ciaglosc predkosci, ciaglosc przyspieszenia
	// ####################################################################################################
	/*
        	double vp[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        	double vk[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        	double ap[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        	double ak[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        	ecp_quintic_generator quintic(tdes, vp, vk, ap, ak);
         	sr_ecp_msg->message("Wykonywany jest ruch o zadany przyrost polozenia/orientacji");
        	Move (*irp6_on_track, NULL, quintic);
         	sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// ###############     KLASA glowna dla odtwarzania listy pozycji     #################################
	// ####################################################################################################
	/*
          teach_in_generator	teach;
          load_file(teach);
          sr_ecp_msg->message("Zaladowano plik");
          teach.initiate_pose_list();
          sr_ecp_msg->message("Zainicjowano liste pozycji");
             // Dlugosc listy = liczba pozycji do odtworzenia
             pll = teach.pose_list_length();
             for (i=0; i< pll; i++) { // Wewnetrzna petla wykonuje sie pll razy (tyle pozycji nauczono)
         	   sr_ecp_msg->message("Wykonywany jest ruch do nastepnej pozycji na licie");
        	   Move (*irp6_on_track, NULL, teach);  // przejscie do nastepnej nauczonej pozycji
         	   sr_ecp_msg->message("Ruch do nastepnej pozycji na licie zakonczyl sie");
        	   }
          sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// Generator odtwarzajacy liste nauczonych pozycji, z rozpedzaniem i hamowaniem miedzy pozycjami,
	// z dokladna zadana pozycja koncowa
	// ####################################################################################################
	/*
        ecp_parabolic_teach_in_generator	pteach;
        	load_file(pteach);
        sr_ecp_msg->message("Zaladowano plik");
        		pteach.initiate_pose_list();
        sr_ecp_msg->message("Zainicjowano liste pozycji");
          		 // Dlugosc listy = liczba pozycji do odtworzenia
           	pll = pteach.pose_list_length();
           		for (i=0; i< pll; i++)
        	{ // Wewnetrzna petla wykonuje sie pll razy (tyle pozycji nauczono)
        			sr_ecp_msg->message("Wykonywany jest ruch do nastepnej pozycji na licie");
          			Move (*irp6_on_track, NULL, pteach);  // przejscie do nastepnej nauczonej pozycji
        			sr_ecp_msg->message("Ruch do nastepnej pozycji na licie zakonczyl sie");
          		}
        sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// Generator odtwarzajacy liste nauczonych pozycji, wykorzystywany do kalibracji
	// ####################################################################################################
	/*
        	ecp_calibration_generator	cal;
         		load_file(cal);
        		sr_ecp_msg->message("Zaladowano plik");
          		cal.initiate_pose_list();
        		sr_ecp_msg->message("Zainicjowano liste pozycji");
            		 // Dlugosc listy = liczba pozycji do odtworzenia
             	pll = cal.pose_list_length();
             		for (i=0; i< pll; i++)
        			{ // Wewnetrzna petla wykonuje sie pll razy (tyle pozycji nauczono)
         				sr_ecp_msg->message("Wykonywany jest ruch do nastepnej pozycji na licie");
        	   			Move (*irp6_on_track, NULL, cal);  // przejscie do nastepnej nauczonej pozycji
         				sr_ecp_msg->message("Ruch do nastepnej pozycji na licie zakonczyl sie");
        	   		}
        	sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// Generator interpolujacy sklejanymi wielomianami 3 stopnia,
	// z rozpedzaniem i hamowaniem miedzy pozycjami
	// ####################################################################################################
	/*
        	cubic_spline_generator	s3;
         		load_file(s3);
        		sr_ecp_msg->message("Zaladowano plik");
          		s3.initiate_pose_list();
        		sr_ecp_msg->message("Zainicjowano liste pozycji");
            		 // Dlugosc listy = liczba pozycji do odtworzenia
             	pll = s3.pose_list_length();
             		for (i=0; i< pll; i++)
        			{ // Wewnetrzna petla wykonuje sie pll razy (tyle pozycji nauczono)
         				sr_ecp_msg->message("Wykonywany jest ruch do nastepnej pozycji na licie");
        				Move (*irp6_on_track, NULL, s3);  // przejscie do nastepnej nauczonej pozycji
         				sr_ecp_msg->message("Ruch do nastepnej pozycji na licie zakonczyl sie");
        			}
        	sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// Generator interpolujacy sklejanymi wielomianami 3 stopnia   (SMOOTH)
	// ####################################################################################################
	/*
        		double vp[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        		double vk[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        	 	smooth_cubic_spline_generator	smooth(vp,vk);
         		load_file(smooth);
         		sr_ecp_msg->message("Zaladowano plik");
         	  		smooth.initiate_pose_list();
         		sr_ecp_msg->message("Zainicjowano liste pozycji");
            		 // Dlugosc listy = liczba pozycji do odtworzenia
             	pll = smooth.pose_list_length();
             		for (i=0; i< pll; i++)
        			{ // Wewnetrzna petla wykonuje sie pll razy (tyle pozycji nauczono)
         		sr_ecp_msg->message("Wykonywany jest ruch do nastepnej pozycji na licie");
        	   			Move (*irp6_on_track, NULL, smooth);  // przejscie do nastepnej nauczonej pozycji
         		sr_ecp_msg->message("Ruch do nastepnej pozycji na licie zakonczyl sie");
        			}
         		sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// Generator interpolujacy sklejanymi wielomianami 5 stopnia,
	// z rozpedzaniem i hamowaniem miedzy pozycjami
	// ####################################################################################################
	/*
        	quintic_spline_generator	s5;
         		load_file(s5);
        		sr_ecp_msg->message("Zaladowano plik");
          		s5.initiate_pose_list();
        		sr_ecp_msg->message("Zainicjowano liste pozycji");
            		 // Dlugosc listy = liczba pozycji do odtworzenia
             	pll = s5.pose_list_length();
             		for (i=0; i< pll; i++)
        			{ // Wewnetrzna petla wykonuje sie pll razy (tyle pozycji nauczono)
         				sr_ecp_msg->message("Wykonywany jest ruch do nastepnej pozycji na licie");
        				Move (*irp6_on_track, NULL, s5);  // przejscie do nastepnej nauczonej pozycji
         				sr_ecp_msg->message("Ruch do nastepnej pozycji na licie zakonczyl sie");
        			}
        	sr_ecp_msg->message("Ruch robota zakonczyl sie");
	 */
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// Informacja dla MP o zakonczeniu zadania uzytkownika
	ecp_termination_notice ();  // ?????????????
}

}
} // namespace irp6ot

namespace common {
namespace task {

base* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::jarosz(_config);
}
} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

