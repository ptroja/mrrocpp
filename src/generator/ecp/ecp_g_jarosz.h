// -------------------------------------------------------------------------
//                            generator/ecp_g_jarosz.h dla QNX6
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

#include "base/lib/impconst.h"

#include "generator/ecp/ecp_g_teach_in.h"
#include "generator/ecp/ecp_g_delta.h"
#include "generator/ecp/ecp_g_operator_reaction_condition.h"

#include <Eigen/Core>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

class linear : public delta
{
  int mp_communication_mode; // by Y - 0 bez TASK TERMINATED, 1 - z TASK TERMINATED
public:
	// konstruktor
	linear (common::task::task& _ecp_task);
	linear (common::task::task& _ecp_task, lib::trajectory_description tr_des, int mp_communication_mode_arg = 1);

  virtual bool first_step ();
  virtual bool next_step ();

}; // end: class all_irp6p_linear_generator






//------------------------------------------------------------------------------
/**
 *  Generator o zadany przyrost polozenia/orientacji.
 * 	Interpolacja funkcja liniowa z parabolicznymi odcinkami krzywoliniowymi,
 * 	czyli trapezoidalny profil predkosci.
 *  W wyniku wykorzystania do realizacji ruchu tego generatora mozliwe jest
 * 	przesuniecie o pewien przyrost polozenia i orientacji, wyrazony we wspolrzednych
 * 	zdefiniowanych w ::lib::POSE_SPECIFICATION, zadany w strukturze ::lib::trajectory_description
 *	Poniewaz czysta interpolacja liniowa moze powodowac nieciaglosci predkosci w
 * 	poczatku i koncu ruchu, aby utworzyc gladka krzywa przemieszczenia i ciagla krzywa
 * 	predkosci nalezy dodac do funkcji liniowej fragmenty paraboli na poczatku i koncu ruchu.
 * 	W przedzialach czasu, ktore odpowiadaja zakrzywionym czesciom trajektorii,
 * 	przyspieszenie jest stale i zapewnia gladka zmiane predkosci. Funkcja liniowa i dwie
 * 	funkcje paraboliczne zostana sklejone razem tak, aby uzyskac ciagle krzywe
 * 	przemieszczen i predkosci.
 */
class linear_parabolic : public delta
{
protected:
  /** Flaga, mowiaca czy jest to pierwszy przedzial interpolacji. */
  int first_interval;
	double ta[lib::MAX_SERVOS_NR];
	double tb[lib::MAX_SERVOS_NR];
	double prev_s[lib::MAX_SERVOS_NR];
	double prev_vel_avg[lib::MAX_SERVOS_NR];

  /**
   *  Metoda sluzy do obliczenia przyrostu drogi.
	 *  Wszystkie czasy znormalizowane sa do 1 (poprzez dzielenie przez liczbe
	 * 	makrokrokow) przy wywolaniu funkcji calculate_s w metodzie next_step.
   *
 	 * 	@return obliczony przyrost drogi
   * 	@param	const double t		aktualny czas
	 * 	@param	const double ta		czas, w ktorym predkosc przestaje rosnac
	 * 	@param	const double tb		czas, w ktorym predkosc zaczyna malec
   * */
	double calculate_s ( const double t , const double ta, const double tb);

public:

  /** Konstruktor domyslny. */
  linear_parabolic (common::task::task& _ecp_task,
                                  lib::trajectory_description tr_des,
                                  const double *time_a ,
                                  const double *time_b
                                 );

  /** Funkcja generujaca pierwszy krok ruchu.
   *  Jest to fukncja wirtualna - opisana zostanie przy kazdej z realizacji
   *  w klasach pochodnych.
   */
  virtual bool first_step ();

  /** Generuje kazdy nastepny krok ruchu.
   *  Jest to fukncja wirtualna - opisana zostanie przy kazdej z realizacji
   *  w klasach pochodnych.
   */
  virtual bool next_step ();

}; // end: ecp_linear_parabolic_generator




// ####################################################################################################
// Klasa bazowa dla generatorow o zadany przyrost polozenia/orientacji
// wykorzystujacych do interpolacji wielomiany
// ####################################################################################################

class polynomial : public delta
{
protected:
   int first_interval;             // flaga, m�wiaca czy jest to pierwszy przedzial interpolacji

public:
   polynomial (common::task::task& _ecp_task);
   virtual bool first_step ();
   virtual bool next_step () = 0;

}; // end: class irp6p_polynomial_generator

// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji wykorzystujacy do interpolacji wielomian 3 stopnia
// ciaglosc predkosci
// predkosc poczatkowa i koncowa moze byc zadawana
// ####################################################################################################

class cubic : public polynomial
{
protected:

   double A0[lib::MAX_SERVOS_NR];                   // parametry wielomianu
   double A1[lib::MAX_SERVOS_NR];
   double A2[lib::MAX_SERVOS_NR];
   double A3[lib::MAX_SERVOS_NR];
   double vp[lib::MAX_SERVOS_NR];
   double vk[lib::MAX_SERVOS_NR];

public:

   // ---------konstruktor dla dla zadanych predkosci vp i vk ---------------
   cubic (common::task::task& _ecp_task, lib::trajectory_description tr_des, double *vp, double *vk);

   virtual bool next_step ();

}; // end: class irp6p_cubic_generator



// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji wykorzystujacy do interpolacji wielomian 5 stopnia
// ciaglosc predkosci, ciaglosc przyspieszenia
// ####################################################################################################
class quintic : public polynomial
{
protected:
   double A0[lib::MAX_SERVOS_NR];		// parametry wielomianu
   double A1[lib::MAX_SERVOS_NR];
   double A2[lib::MAX_SERVOS_NR];
   double A3[lib::MAX_SERVOS_NR];
   double A4[lib::MAX_SERVOS_NR];
   double A5[lib::MAX_SERVOS_NR];
   double vp[lib::MAX_SERVOS_NR];		// predkosci
   double vk[lib::MAX_SERVOS_NR];
   double ap[lib::MAX_SERVOS_NR];		// przyspieszenia
   double ak[lib::MAX_SERVOS_NR];

public:

	// --------- konstruktor dla dla zadanych predkosci vp i vk i przysp. ap i ak -------------
   quintic (common::task::task& _ecp_task, lib::trajectory_description tr_des, double *vp, double *vk, double *ap, double *ak);

   virtual bool next_step ();

}; // end: class irp6p_quintic_generator



// ####################################################################################################
// ################################     KLASA BAZOWA dla SPLAJNOW     #################################
// ####################################################################################################

class spline : public teach_in
{
protected:
  ecp_taught_in_pose tip;			// Kolejna pozycja.
									// Trajektoria miedzy kolejnymi
									// pozycjami dzielona jest na przedzialy interpolacji.
									// Kazdy przedzial interpolacji jest rownowazny jednemu makrokrokowi
									// wykonywanemu przez EDP.
  bool first_interval;		// true  - jezeli ruch odbywa sie w pierwszym przedziale interpolacji
									// false	- w przeciwnym przypadku
  unsigned int number_of_intervals;      // Liczba przedzialow interpolacji - wynika z calkowitego czasu ruchu
  double INTERVAL;				// Dlugosc okresu interpolacji w [sek]
  double a_max_motor[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_motor[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_joint[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_joint[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_zyz[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_zyz[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_aa[lib::MAX_SERVOS_NR];			// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_aa[lib::MAX_SERVOS_NR];			// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych

  virtual bool first_step ()=0;
  virtual bool next_step ()=0;

  public:
   spline (common::task::task& _ecp_task);

}; // end : irp6p_spline_generator

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, z rozpedzaniem i hamowaniem miedzy pozycjami,
// z dokladna zadana pozycja koncowa
// ####################################################################################################

class parabolic_teach_in : public spline
{
protected:

  unsigned int half_number_of_intervals;								// Polowa liczby przedzialow interpolacji
  double a[lib::MAX_SERVOS_NR];													// tablica faktycznie realizowanych przyspieszen
																	// dla kolejnych osi/wspolrzednych
public:
	  parabolic_teach_in (common::task::task& _ecp_task, double interval);

  	virtual bool first_step ();
  	virtual bool next_step ();

}; // end: irp6p_parabolic_teach_in_generator

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, wykorzystywany do kalibracji
// ####################################################################################################

class calibration : public spline
{
protected:

  unsigned int half_number_of_intervals;								// Polowa liczby przedzialow interpolacji
  double a[lib::MAX_SERVOS_NR];													// tablica faktycznie realizowanych przyspieszen

public:
  calibration (common::task::task& _ecp_task, double interval);
	// Zapis danych z kalibracji do pliku
	void ecp_save_extended_file (operator_reaction_condition& the_condition);

   virtual bool first_step ();
   virtual bool next_step ();

}; // end: class irp6p_calibration_generator

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 3 stopnia,
// z rozpedzaniem i hamowaniem miedzy pozycjami
// ####################################################################################################

class cubic_spline : public spline
{
protected:

   double A0[lib::MAX_SERVOS_NR];                   // Parametry wielomianu
   double A2[lib::MAX_SERVOS_NR];                   // A1[i] = 0
   double A3[lib::MAX_SERVOS_NR];

public:


	  cubic_spline();
	  cubic_spline (common::task::task& _ecp_task, double interval);

   virtual bool first_step ();
   virtual bool next_step ();

}; // end: irp6p_cubic_spline_generator


// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 3 stopnia
// ####################################################################################################

class smooth_cubic_spline : public spline
{
protected:

   int j;							// Licznik pozycji
   bool build_coeff;				// Flaga oznaczajaca pierwsze uzycie generatora
   double vp[lib::MAX_SERVOS_NR];					// Tablica predkosci poczatkowych
   double vk[lib::MAX_SERVOS_NR]; 					// Tablica predkosci koncowych

   Eigen::MatrixXd y, t, a;					// Macierze: czasow, polozen i przyspieszen - w punktach wezlowych

  void Build_Coeff (double *tt, double *yy, int nn, double vvp, double vvk, double *aa);

public:
	  smooth_cubic_spline (common::task::task& _ecp_task, double *vp, double *vk, double interval);

   virtual bool first_step ();
   virtual bool next_step ();

}; // end: irp6p_smooth_cubic_spline_generator

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 5 stopnia,
// z rozpedzaniem i hamowaniem miedzy pozycjami
// ####################################################################################################

class quintic_spline : public spline
{
protected:

   double A0[lib::MAX_SERVOS_NR];                 // Parametry wielomianu
   double A3[lib::MAX_SERVOS_NR];                 // A1[i] = 0 , A2[i]=0
   double A4[lib::MAX_SERVOS_NR];
   double A5[lib::MAX_SERVOS_NR];

public:
	  quintic_spline (common::task::task& _ecp_task, double interval);

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
    double coordinates[lib::MAX_SERVOS_NR]; // wspolrzedne
  };


// ####################################################################################################
// Generator odtwarzajacy trajektorie zadana analitycznie - elipsa
// ####################################################################################################

class elipsoid : public teach_in {

protected:
  ecp_taught_in_pose tip;			// Kolejna z listy nauczonych pozycji.
								// Trajektoria miedzy kolejnymi nauczonymi
								// pozycjami dzielona jest na przedzialy interpolacji.
								// Kazdy przedzial interpolacji jest rownowazny jednemu makrokrokowi
								// wykonywanemu przez EDP.
  one_sample *trj_ptr;			// wskaznik na bufor z rzeczywista trajektoria

  bool first_interval;		// true  - jezeli ruch odbywa sie w pierwszym przedziale interpolacji
								// false	- w przeciwnym przypadku

  unsigned int number_of_intervals;		// Liczba przedzialow interpolacji - wynika z calkowitego czasu ruchu
  unsigned int half_number_of_intervals;	// Polowa liczby przedzialow interpolacji

  double next_pose[lib::MAX_SERVOS_NR];			// Nastepna pozycja liscie - punkt koncowy trajktorii parabolicznej
  double a[lib::MAX_SERVOS_NR];					// tablica faktycznie realizowanych przyspieszen dla kolejnych osi/wspolrzednych

  double INTERVAL;				// Dlugosc okresu interpolacji w [sek]
  double a_max_motor[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_motor[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_joint[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_joint[lib::MAX_SERVOS_NR];		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_zyz[lib::MAX_SERVOS_NR];			// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_zyz[lib::MAX_SERVOS_NR];			// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
  double a_max_aa[lib::MAX_SERVOS_NR];			// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
  double v_max_aa[lib::MAX_SERVOS_NR];			// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych

public:
	  elipsoid (common::task::task& _ecp_task);


	  // Zapis rzeczywistej trajektorii do pliku
	  void ecp_save_trajectory ();

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




// --------------------------------------------------------------------------


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
