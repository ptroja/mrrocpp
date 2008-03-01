// -------------------------------------------------------------------------
//                            ecp_g_jarosz.h dla QNX6
// 
// Modyfikacje:
// 1. metody wirtualne w kla sie bazowej sensor - ok. 160
// 2. bonusy do testowania
// 
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_JAROSZ_H)
#define _ECP_GEN_JAROSZ_H

#include "common/impconst.h"

#include "lib/matrix.h"
#include "ecp/common/ecp_teach_in_generator.h"
#include "ecp/common/ecp_operator_reaction_condition.h"

// ########################################################################################################
// ########################################################################################################
// ########################### GENERATORY RUCHU DLA ECP (opracowane by Jarosz) ############################
// ########################################################################################################
// ########################################################################################################

// ####################################################################################################
// KLASA BAZOWA dla generatorow o zadany przyrost polozenia/orientacji  
// ####################################################################################################

class ecp_delta_generator : public ecp_generator 
{
protected:

  
  double a_max_motor[MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_motor[MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_joint[MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_joint[MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_zyz[MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_zyz[MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_aa[MAX_SERVOS_NR];			// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_aa[MAX_SERVOS_NR];			// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych

public:	
	ecp_delta_generator(ecp_task& _ecp_task);

   trajectory_description td;   

   virtual bool first_step () = 0;	
   virtual bool next_step () = 0;	

}; // end : class irp6p_ECP_delta_generator

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji  
// ####################################################################################################

class ecp_linear_generator : public ecp_delta_generator 
{
protected:
	int mp_communication_mode; // by Y - 0 bez TASK TERMINATED, 1 - z TASK TERMINATED
	bool finished;
	
public:
	// konstruktor
	ecp_linear_generator (ecp_task& _ecp_task);
	ecp_linear_generator (ecp_task& _ecp_task, trajectory_description tr_des, int mp_communication_mode_arg = 1);
	
  virtual bool first_step ();
  virtual bool next_step ();

}; // end: class all_irp6p_linear_generator



// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji
// Interpolacja funckja liniowa z parabolicznymin odcinkami krzykowliniowymi,
// czyli trapezoidalny profil predkosci
// ####################################################################################################

class ecp_linear_parabolic_generator : public ecp_delta_generator
{
protected:
     int first_interval;             // flaga, mówiaca czy jest to pierwszy przedzial interpolacji
	double calculate_s ( const double t , const double ta, const double tb);
	double ta[MAX_SERVOS_NR];
	double tb[MAX_SERVOS_NR];

	double prev_s[MAX_SERVOS_NR];
	double prev_vel_avg[MAX_SERVOS_NR];
		
public:	

 // ------- konstruktor -------------
 ecp_linear_parabolic_generator (ecp_task& _ecp_task, trajectory_description tr_des, const double *time_a , const double *time_b);	   
   
  virtual bool first_step ();
  virtual bool next_step ();

}; // end: class irp6p_+linear_parabolic_generator

// ####################################################################################################
// Klasa bazowa dla generatorow o zadany przyrost polozenia/orientacji 
// wykorzystujacych do interpolacji wielomiany
// ####################################################################################################

class ecp_polynomial_generator : public ecp_delta_generator 
{
protected:
   int first_interval;             // flaga, mówiaca czy jest to pierwszy przedzial interpolacji
  
public:	
   ecp_polynomial_generator (ecp_task& _ecp_task);
   virtual bool first_step ();	
   virtual bool next_step () = 0;	 

}; // end: class irp6p_polynomial_generator

// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji wykorzystujacy do interpolacji wielomian 3 stopnia
// ciaglosc predkosci
// predkosc poczatkowa i koncowa moze byc zadawana 
// ####################################################################################################

class ecp_cubic_generator : public ecp_polynomial_generator 
{
protected:

   double A0[MAX_SERVOS_NR];                   // parametry wielomianu
   double A1[MAX_SERVOS_NR]; 
   double A2[MAX_SERVOS_NR]; 
   double A3[MAX_SERVOS_NR];
   double vp[MAX_SERVOS_NR];
   double vk[MAX_SERVOS_NR];

public:
   
   // ---------konstruktor dla dla zadanych predkosci vp i vk ---------------
   ecp_cubic_generator (ecp_task& _ecp_task, trajectory_description tr_des, double *vp, double *vk); 		 
   
   virtual bool next_step ();	 

}; // end: class irp6p_cubic_generator



// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji wykorzystujacy do interpolacji wielomian 5 stopnia
// ciaglosc predkosci, ciaglosc przyspieszenia
// ####################################################################################################
class ecp_quintic_generator : public ecp_polynomial_generator 
{
protected:
   double A0[MAX_SERVOS_NR];		// parametry wielomianu
   double A1[MAX_SERVOS_NR]; 
   double A2[MAX_SERVOS_NR]; 
   double A3[MAX_SERVOS_NR];
   double A4[MAX_SERVOS_NR];
   double A5[MAX_SERVOS_NR];
   double vp[MAX_SERVOS_NR];		// predkosci		
   double vk[MAX_SERVOS_NR];
   double ap[MAX_SERVOS_NR];		// przyspieszenia
   double ak[MAX_SERVOS_NR];

public:	

	// --------- konstruktor dla dla zadanych predkosci vp i vk i przysp. ap i ak -------------
   ecp_quintic_generator (ecp_task& _ecp_task, trajectory_description tr_des, double *vp, double *vk, double *ap, double *ak);

   virtual bool next_step ();	 

}; // end: class irp6p_quintic_generator



// ####################################################################################################
// ################################     KLASA BAZOWA dla SPLAJNOW     #################################
// ####################################################################################################

class ecp_spline_generator : public ecp_teach_in_generator 
{
protected:
  ecp_taught_in_pose tip;			// Kolejna pozycja.
									// Trajektoria miedzy kolejnymi
									// pozycjami dzielona jest na przedzialy interpolacji.
									// Kazdy przedzial interpolacji jest rownowazny jednemu makrokrokowi
									// wykonywanemu przez EDP.
  bool first_interval;		// true  - jezeli ruch odbywa sie w pierwszym przedziale interpolacji
									// false	- w przeciwnym przypadku
  int number_of_intervals;      // Liczba przedzialow interpolacji - wynika z calkowitego czasu ruchu
  double INTERVAL;				// Dlugosc okresu interpolacji w [sek]
  double a_max_motor[MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_motor[MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_joint[MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_joint[MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_zyz[MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_zyz[MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_aa[MAX_SERVOS_NR];			// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_aa[MAX_SERVOS_NR];			// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych

  virtual bool first_step ()=0;
  virtual bool next_step ()=0;
  
  public:
   ecp_spline_generator (ecp_task& _ecp_task);

}; // end : irp6p_spline_generator

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, z rozpedzaniem i hamowaniem miedzy pozycjami, 
// z dokladna zadana pozycja koncowa
// ####################################################################################################

class ecp_parabolic_teach_in_generator : public ecp_spline_generator
{
protected:

  int half_number_of_intervals;								// Polowa liczby przedzialow interpolacji
  double a[MAX_SERVOS_NR];													// tablica faktycznie realizowanych przyspieszen 
																	// dla kolejnych osi/wspolrzednych
public:
	  ecp_parabolic_teach_in_generator (ecp_task& _ecp_task, double interval);

  	virtual bool first_step ();
  	virtual bool next_step ();

}; // end: irp6p_parabolic_teach_in_generator

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, wykorzystywany do kalibracji
// ####################################################################################################

class ecp_calibration_generator : public ecp_spline_generator 
{
protected:

  int half_number_of_intervals;								// Polowa liczby przedzialow interpolacji
  double a[MAX_SERVOS_NR];													// tablica faktycznie realizowanych przyspieszen 

public:	
	  ecp_calibration_generator (ecp_task& _ecp_task, double interval);
  
   virtual bool first_step ();
   virtual bool next_step ();

}; // end: class irp6p_calibration_generator

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 3 stopnia, 
// z rozpedzaniem i hamowaniem miedzy pozycjami
// ####################################################################################################

class ecp_cubic_spline_generator : public ecp_spline_generator
{
protected:

   double A0[MAX_SERVOS_NR];                   // Parametry wielomianu
   double A2[MAX_SERVOS_NR];                   // A1[i] = 0
   double A3[MAX_SERVOS_NR]; 

public:	
	  ecp_cubic_spline_generator();
	  ecp_cubic_spline_generator (ecp_task& _ecp_task, double interval);

   virtual bool first_step ();	 
   virtual bool next_step ();		 

}; // end: irp6p_cubic_spline_generator


// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 3 stopnia
// ####################################################################################################

class ecp_smooth_cubic_spline_generator : public ecp_spline_generator
{
protected:

   int j;							// Licznik pozycji
   bool build_coeff;				// Flaga oznaczajaca pierwsze uzycie generatora
   double vp[MAX_SERVOS_NR];					// Tablica predkosci poczatkowych
   double vk[MAX_SERVOS_NR]; 					// Tablica predkosci koncowych
   
   matrix y, t, a;					// Macierze: czasow, polozen i przyspieszen - w punktach wezlowych
   
  void Build_Coeff (double *tt, double *yy, int nn, double vvp, double vvk, double *aa);	     

public:
	  ecp_smooth_cubic_spline_generator (ecp_task& _ecp_task, double *vp, double *vk, double interval);

   virtual bool first_step ();		     
   virtual bool next_step ();			     

}; // end: irp6p_smooth_cubic_spline_generator

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 5 stopnia, 
// z rozpedzaniem i hamowaniem miedzy pozycjami
// ####################################################################################################

class ecp_quintic_spline_generator : public ecp_spline_generator
{
protected:

   double A0[MAX_SERVOS_NR];                 // Parametry wielomianu
   double A3[MAX_SERVOS_NR];                 // A1[i] = 0 , A2[i]=0
   double A4[MAX_SERVOS_NR]; 
   double A5[MAX_SERVOS_NR]; 

public:	
	  ecp_quintic_spline_generator (ecp_task& _ecp_task, double interval);

   virtual bool first_step ();		 
   virtual bool next_step ();			 

}; // end: irp6p_quintic_spline_generator

// ####################################################################################################


// ####################################################################################################
// ###################################       INNE GENERATORY RUCHU             ########################
// ####################################################################################################

// Jedna probka pomiarowa (punkt)
  struct one_sample {     // Pojedynczy pomiar
    double ctime;         // czas
    double coordinates[MAX_SERVOS_NR]; // wspolrzedne
  };


// ####################################################################################################
// Generator odtwarzajacy trajektorie zadana analitycznie - elipsa
// ####################################################################################################

class ecp_elipsoid_generator : public ecp_teach_in_generator {

protected:
  ecp_taught_in_pose tip;			// Kolejna z listy nauczonych pozycji.
								// Trajektoria miedzy kolejnymi nauczonymi
								// pozycjami dzielona jest na przedzialy interpolacji.
								// Kazdy przedzial interpolacji jest rownowazny jednemu makrokrokowi
								// wykonywanemu przez EDP.
  one_sample *trj_ptr;			// wskaznik na bufor z rzeczywista trajektoria

  bool first_interval;		// true  - jezeli ruch odbywa sie w pierwszym przedziale interpolacji
								// false	- w przeciwnym przypadku
  
  int number_of_intervals;		// Liczba przedzialow interpolacji - wynika z calkowitego czasu ruchu
  int half_number_of_intervals;	// Polowa liczby przedzialow interpolacji

  double next_pose[MAX_SERVOS_NR];			// Nastepna pozycja liscie - punkt koncowy trajktorii parabolicznej
  double a[MAX_SERVOS_NR];					// tablica faktycznie realizowanych przyspieszen dla kolejnych osi/wspolrzednych

  double INTERVAL;				// Dlugosc okresu interpolacji w [sek]
  double a_max_motor[MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_motor[MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_joint[MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_joint[MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_zyz[MAX_SERVOS_NR];			// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_zyz[MAX_SERVOS_NR];			// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_aa[MAX_SERVOS_NR];			// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_aa[MAX_SERVOS_NR];			// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych

public:
	  ecp_elipsoid_generator (ecp_task& _ecp_task);


  virtual bool first_step ();
      // generuje pierwszy krok ruchu -
      // pierwszy krok czesto rozni sie od pozostalych,
      // np. do jego generacji nie wykorzystuje sie czujnikow
      // (zadanie realizowane przez klase konkretna)
  virtual bool next_step ();
     // generuje kazdy nastepny krok ruchu
     // (zadanie realizowane przez klase konkretna)

  int get_number_of_intervals (void) {return number_of_intervals;};

  void get_sample (one_sample& cp, int sn );

  void clear_buffer (void);
}; // end: irp6p_elipsoid_generator

// ########################################################################################################
// ####################################    KONIEC GENERATOROW   ###########################################
// ########################################################################################################


// Zapis rzeczywistej trajektorii do pliku
void ecp_save_trajectory (ecp_elipsoid_generator& the_generator, ecp_task& _ecp_task);

// --------------------------------------------------------------------------
// Zapis danych z kalibracji do pliku
void ecp_save_extended_file (ecp_calibration_generator& the_generator,
	 ecp_operator_reaction_condition& the_condition, ecp_task& _ecp_task);

#endif
