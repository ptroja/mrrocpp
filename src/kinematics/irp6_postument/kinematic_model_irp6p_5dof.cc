// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6p_5dof.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- definicja metod klasy
//				- kinematyka 5stopniowa
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

#include <stdlib.h>
#include <math.h>

#include "lib/com_buf.h"

// Klasa kinematic_model_irp6p_5dof.
#include "kinematics/irp6_postument/kinematic_model_irp6p_5dof.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6p {

/* -----------------------------------------------------------------------
  Konstruktor.
 ------------------------------------------------------------------------- */
model_5dof::model_5dof (int _number_of_servos):
	model_with_wrist(_number_of_servos)
{
  // Ustawienie etykiety modelu kinematycznego.
  set_kinematic_model_label("Switching to 5 DOF kinematic model");

  // Ustawienie parametrow kinematycznych.
  // Podstawowe parametry ustawione przez sa konstruktor with_wrist.
  set_kinematic_parameters();

} // end: kinematic_model::kinematic_model


void model_5dof::set_kinematic_parameters(void)
{
  // Wysokosc kolumny.
  d1 = 0.7;
  // Modyfikacja pozostalych
  synchro_motor_position[4]= 314.665;		// kisc V [rad]
  synchro_joint_position[4] = synchro_motor_position[4] - gear[4] * theta[4] - synchro_motor_position[3];

  // Zmiana ustawienia czesci standardowego narzedzia na osiowo-symetryczne [m].
  tool.set_translation_vector(0, 0, 0);

}//: set_kinematic_parameters


/* ------------------------------------------------------------------------
  Przeliczenie polozenia koncowki zwiazane z dolaczonym narzedziem - transformacja odwrotna.
  Poniewaz narzedzie jest uzwane podczas rozwiazywania wlasciwego odwrotnego zadania kinematyki,
  metoda musi byc pusta - przedefiniowanie standardowej metody.
  W prostym zadaniu narzedzue jest "dolaczone" normalnie.
 ------------------------------------------------------------------------ */
//void model_5dof::attached_tool_inverse_transform(lib::Homog_matrix& local_current_end_effector_frame)
//{
//	return;
//}//: attached_tool_inverse_transform


/* ------------------------------------------------------------------------
  Zadanie proste kinematyki dla robota IRp-6 na postumencie.
  Kinematyka pieciostopniowa.

  Wejscie:
  * current_joints[6] - wspolrzedne wewnetrzne robota (kolejno q0, q1, q2, ...)

  Wyjscie:
  * current_end_effector_frame[4][3] - macierz przeksztacenia jednorodnego (MPJ)
		opisujca aktualne poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.
 ------------------------------------------------------------------------ */
void model_5dof::direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{

  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
  check_joints (local_current_joints);

  // Parametry pomocnicze.
  double s1 = sin(local_current_joints[0]);
  double c1 = cos(local_current_joints[0]);
  double s2 = sin(local_current_joints[1]);
  double c2 = cos(local_current_joints[1]);
  double s3 = sin(local_current_joints[2]);
  double c3 = cos(local_current_joints[2]);
  double s4 = sin(local_current_joints[3]);
  double c4 = cos(local_current_joints[3]);
  double s5 = sin(local_current_joints[4]);
  double c5 = cos(local_current_joints[4]);

  // Proste zadanie kinematyki.
  local_current_end_effector_frame(0,0) = (c5*s4*c1+s1*s5);
  local_current_end_effector_frame(0,1) = (-s5*s4*c1+s1*c5);
  local_current_end_effector_frame(0,2) = c4*c1;
  local_current_end_effector_frame(0,3) = c1*(d6*c4+a3*c3+a2*c2);
  local_current_end_effector_frame(1,0) = (c5*s4*s1-c1*s5);
  local_current_end_effector_frame(1,1) = (-s5*s4*s1-c1*c5);
  local_current_end_effector_frame(1,2) = c4*s1;
  local_current_end_effector_frame(1,3) = d6*c4*s1 + a3*c3*s1 + a2*s1*c2;
  local_current_end_effector_frame(2,0) = (c4*c5);
  local_current_end_effector_frame(2,1) = (-c4*s5);
  local_current_end_effector_frame(2,2) = -s4;
  local_current_end_effector_frame(2,3) = -d6*s4 - a3*s3 - a2*s2 + d1;

} //:: i2e_transform()

/* ------------------------------------------------------------------------
  Zadanie odwrotne kinematyki dla robota IRp-6 na postumencie.
  Kinematyka pieciostopniowa.

  Wejscie:
  * local_current_joints - obecne (w rzeczywistosci poprzednie) wspolrzedne wewnetrzne robota (kolejno q0, q1, q2, ...)
  * local_desired_end_effector_frame - macierz przeksztacenia jednorodnego (MPJ)
		opisujca zadane poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.

  Wyjscie:
  * local_desired_joints - wyliczone wspolrzedne wewnetrzne robota (kolejno q0, q1, q2, ...)
 ------------------------------------------------------------------------ */
void model_5dof::inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
  // Stale
 // const double a2_2 = a2*a2;
 // const double a3_2 = a3*a3;
//  const double EPS=1e-10;
  // Zmienne pomocnicze.

  // Obliczenia - budowanie drzewa rozwiazan.

  double q0[3];
  double v0[3];
  double q6[3];
  double v6[3];
  double old_theta[5];
  double interpolation_period;
  double Theta[5];
  double tsin[5];
  double tcos[5];

  interpolation_period = INTER_PERIOD;

  // Przepisanie starych wartosci katow.
  for(int i = 0; i < 5; i++)
	old_theta[i] = local_current_joints[i];


  q0[0] = local_desired_end_effector_frame(0,3);
  q0[1] = local_desired_end_effector_frame(1,3);
  q0[2] = local_desired_end_effector_frame(2,3);

  // Pobranie danych narzedzia.
  lib::Homog_matrix tmp_tool_m = tool;

  q6[0] = tmp_tool_m(0,3);
  q6[1] = tmp_tool_m(1,3);
  q6[2] = tmp_tool_m(2,3);

  v0[0] = local_desired_end_effector_frame(0,0);
  v0[1] = local_desired_end_effector_frame(1,0);
  v0[2] = local_desired_end_effector_frame(2,0);// tutaj byl minus, ale dlaczego???

  v6[0] = tmp_tool_m(0,0);
  v6[1] = tmp_tool_m(1,0);
  v6[2] = tmp_tool_m(2,0);

  THETA_NODE *tree_ptr;   /* wskaznik korzenia drzewa rozwiaza`n */
  int16_t res;      /* wynik dzialania Theta_1 - OK lub nie */

  tree_ptr = Theta_1( q0, v0, q6, v6, &res, old_theta, interpolation_period, SINGLE_SOLUTION);

  if(res == OK)
  {

      Extract_vect_from_tree(tree_ptr, Theta, tsin, tcos);
      /* Skasowanie drzewa rozwiaza`n */
      Delete_Theta_Tree(tree_ptr);

	local_desired_joints[0] = Theta[0];
	local_desired_joints[1] = Theta[1];
	local_desired_joints[2] = Theta[2];
 	local_desired_joints[3] = Theta[3];
	local_desired_joints[4] = Theta[4];
  }
  else
  {

	local_desired_joints[0] = Theta[0];
	local_desired_joints[1] = Theta[1];
	local_desired_joints[2] = Theta[2];
	local_desired_joints[3] = Theta[3];
	local_desired_joints[4] = Theta[4];

	for (int i=0;i<5;i++)
		std::cout<<"e2i("<<i<<") = "<<local_desired_joints[i]<<"\t";
	std::cout<<std::endl;
	}

  // Przepisanie ostatniego kata - staly.
  local_desired_joints[5] = local_current_joints[5];

  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
  check_joints (local_desired_joints);

} //: inverse_kinematics_transform()



/***************************************************************************
*                                                                          *
*                              Theta_1                                     *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Theta_1 oblicza kat THETA1 w trakcie rozwiazywania odwrotnego
  zagadnienia kinematycznego dla robota IRp-6. Tworzy wezly THETA1
  drzewa rozwiaza`n oraz wywoluje funkcje obliczajace pozostale
  THETA dla dopuszczalnych THETA1. Funkcje te tworza adekwatne poddrzewa
  rozwiaza`n.


WYWOlANIE:

   ptr = Theta_1(q0, v0, q6, v6, result, old_theta,
		 interpolation_period, no_of_solutions);


DANE WEJSCIOWE:

  double q0[3] - wektor (zwiazany) polozenia ko`nca narzedzia (TCP)
		 wzgledem bazowego ukladu odniesienia
  double q6[3] - wektor (zwiazany) polozenia ko`nca narzedzia (TCP)
		 wzgledem ukladu wspolrzednych zwiazanego z kolnierzem
  double v0[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem bazowego ukladu odniesienia
  double v6[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem ukladu wspolrzednych zwiazanego z kolnierzem

  double old_theta[5]   - poprzednia wartosc wejsciowego wektora
			  wspolrzednych wewnetrznych

  double interpolation_period   - okres makrointerpolacji w [ms]

  int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				  byc obliczone wszystkie rozwiazania, czy
				  tez jedynie to lezace najblizej
				  poprzedniego


DANE WYJSCIOWE:

  Wartosc funkcji:
   THETA_NODE *ptr - wskaznik na wezel drzewa rozwiaza`n zawierajacy
		      pierwsza dopuszczalna wartosc THETA1 lub NULL,
		      gdy jej brak.

    int16_t *result - czy wynik dzialania poprawny

    *result = OK - stworzono drzewo rozwiaza`n.
    *result = NEGATIVE_DISCRIMINANT_THETA1
    *result = INCONSISTENT_DATA_THETA1
    *result = UNKNOWN_ERROR_THETA1
    *result = INVALID_ROOT_THETA1
    *result = OUT_OF_MEMORY
    *result = INVALID_ROOTS_THETA1
    *result = bledy zwracane przez funkcje wywolywane przez Theta_1


ZMIENNE ZEWNeTRZNE:

  Drzewo rozwiaza`n odwrotnego zagadnienia kinematycznego dla robota IRp-6.
  Znajduje sie na stercie (ang. heap). Dostep poprzez wskazniki lokalne.

---------------------------------------------------------------------------*/

#define EPSSS  1.0E-10

THETA_NODE* model_5dof::Theta_1(double q0[3], double v0[3],
		    double q6[3], double v6[3],
		    int16_t *result,
		    double old_theta[5],
		    double interpolation_period,
		    int16_t no_of_solutions)
{
   double u0[3];       /* wsp. srodka okregu wzgledem ukl. bazowego */
   double u6[3];       /* wsp. srodka okregu wzgledem ukl. kolnierza */
   double radius_2;    /* kwadrat promienia okregu */
   double temp1;       /* zmienna pomocnicza */
   double temp2;       /* zmienna pomocnicza */
   int16_t i;           /* licznik petli */
   double sum;         /* sumator */
   double x1;          /* zmienna pomocnicza przeznaczona na przechowywanie
			  pierwszego rozwiaza`n rownania kwadratowego */
   double x2;          /* zmienna pomocnicza przeznaczona na przechowywanie
			  drugiego rozwiazania rownania kwadratowego */
   int16_t res;         /* zmienna pomocnicza przeznaczona na przechowywanie
			  rodzaju rozwiaza`nia rownania kwadratowego */
   double p;           /* zmienna pomocnicza wykorzystywana przy obliczaniu
			  wspolczynnikow rownania kwadratowego */
   double r;           /* zmienna pomocnicza wykorzystywana przy obliczaniu
			  wspolczynnikow rownania kwadratowego */
   double t;           /* zmienna pomocnicza wykorzystywana przy obliczaniu
			  wspolczynnikow rownania kwadratowego */
   double r_2;         /* r*r */
   double delta;       /* wyroznik rownania kwadratowego */
   double max_theta1_inc;     /* maksymalny dopuszczalny przyrost ka`ta
				 THETA w 1 kroku makro-interpolacji */
   THETA_NODE *root_pointer;  /* wskaznik na korze`n w poddrzewie
				 rozwiaza`n */
   THETA_NODE *node_pointer;  /* wskaznik na wezel w drzewie
				 rozwiaza`n  */
   THETA_NODE *temp_pointer;  /* wskaznik na wezel w poddrzewie
				 rozwiaza`n  */
   THETA_NODE *tmp_ptr;       /* wskaznik na wezel w drzewie
				 rozwiaza`n  */
   THETA_NODE *previous;     /* wskaznik na poprzedni wezel tego samego
				kata w drzewie rozwiaza`n  */


/* Poczatek Theta_1() */
// // std::cout << "Jestem w Theta_1" << std::endl;

/* inicjalizacja wskaznikow */
   root_pointer = NULL;
   node_pointer = NULL;

    *result = OK;

/* obliczenie maksymalnego dopuszczalnego przyrostu kata THETA w jednym
   kroku makro-interpolacji */
   max_theta1_inc = interpolation_period * MAX_THETA1_VELOCITY / 1000.0;

  /* obliczenie u6 i u0 oraz kwadratu promienia */
   sum = 0.0;
   for(i=X;i<=Z;i++)
     sum += v6[i]*q6[i];

   for(i=X;i<=Z;i++)
    {
     u6[i] = q6[i] - v6[i]*sum;
     u0[i] = q0[i] - v0[i]*sum;
    }

    radius_2 = 0.0;
    for(i=X;i<=Z;i++)
      radius_2 += u6[i]*u6[i];


/* Obliczamy THETA1 */

  temp1 = v0[X]*v0[X] - v0[Y]*v0[Y];
  temp2 = 1.0 - v6[Z]*v6[Z];
  if((v6[Z]>-1.0+EPSSS) && (v6[Z]<1.0-EPSSS)){
		     /* wzor (4.12) */
		     p = u6[Z]*u6[Z]*temp1 - temp2*(u0[X]*u0[X] - u0[Y]*u0[Y]) +
			 2.0*u6[Z]*v6[Z]*(u0[Y]*v0[Y] - v0[X]*u0[X]) - temp1*radius_2;

		     /* wzor (4.13) */
		     r = 2.0*(u6[Z]*u6[Z]*v0[X]*v0[Y] - u0[Y]*u0[X]*temp2 -
			   u6[Z]*v6[Z]*(v0[X]*u0[Y] + u0[X]*v0[Y]) - v0[X]*v0[Y]*radius_2);

		     /* wzor (4.14) */
		     t = (u6[Z]*u6[Z] - radius_2)*(v0[Z]*v0[Z] + v0[Y]*v0[Y]) +
			  u0[X]*u0[X]*temp2 + radius_2*v6[Z]*v6[Z] +
			  2.0*u6[Z]*v6[Z]*u0[X]*v0[X];

		      r_2 = r*r;
		      delta = r_2*(r_2 - 4.0*t*(p + t));

		     /* stworzenie warstwy THETA1 w drzewie rozwiaza`n */
		     res = QuadraticEquation(p*p+r_2, 2*p*t-r_2, t*t, &x1, &x2, delta);
    }
   else{
		     temp1 = v0[X]*v0[X] + v0[Y]*v0[Y];
		     if(temp1>EPSSS)
		     {
			       p = v0[Y]*v0[Y] - v0[X]*v0[X];
			       r = -2.0*v0[X]*v0[Y];
			       t = v0[X]*v0[X];
			       x1 = v0[X]*v0[X]/temp1;
			       res = ONE_SOLUTION;
		      }
		      else
		      {
			       res = TRIVIAL_EQUATION;
		      } /* end: if(temp1>EPSSS) */
     }  /* end: if((v6[Z]>-1.0+EPSSS) && (v6[Z]<1.0-EPSSS)) */


  switch(res)
   {
     case NO_SOLUTION:
	  *result = NEGATIVE_DISCRIMINANT_THETA1; return(NULL);
     case INCONSISTENT_DATA:
	  *result = INCONSISTENT_DATA_THETA1; return(NULL);
     case ONE_SOLUTION:
//      std::cout << " Jedno rozwiazanie one_solution ";
     case LINEAR_SOLUTION:
//      std::cout << " Jedno rozwiazanie LINEAR_SOLUTION ";
			  root_pointer = Create1(x1,p,r,t,result,max_theta1_inc,no_of_solutions,old_theta[0]);

			  switch (*result)
			   {
			    case OUT_OF_MEMORY: return(NULL);
			    case OUT_OF_RANGE:
			    case INVALID_ROOT_X: *result = INVALID_ROOT_THETA1; return(NULL);
			    case ONE_NODE:
			    case TWO_NODES: break;
			    default : *result = UNKNOWN_ERROR_THETA1; return(NULL);
			   } /* end switch (*result) */
			   break;
     case TWO_SOLUTIONS:
	//  	   std::cout << "dwa rozwiazania";

	       /* pierwsze rozwiazanie */
	       // std::cout << "przed Create1" << std::endl;
	       // std::cout << "x1 = " << x1 << std::endl;
	       // std::cout << "p = " << p << std::endl;
	       // std::cout << "r = " << r << std::endl;
	       // std::cout << "t = " << t << std::endl;
	//      // std::cout << "result = " << result << std::endl;

		  root_pointer = Create1(x1,p,r,t,result,max_theta1_inc,no_of_solutions,old_theta[0]);
		  switch (*result)
		   {
			    case OUT_OF_MEMORY: return(NULL);
			    case OUT_OF_RANGE:
			    case INVALID_ROOT_X:
			    case ONE_NODE:
			    case TWO_NODES: break;
			    default : *result = UNKNOWN_ERROR_THETA1; return(NULL);
		   } /* end switch (*result) */

	      	/* drugie rozwiazanie */
			// std::cout << "drugie rozwiazanie" <<std::endl;
		  if(root_pointer == NULL){
			  // std::cout << "x2 = " << x2 << std::endl;
			  // std::cout << "root_pointer byl null"<<std::endl;
			    root_pointer = Create1(x2,p,r,t,result,max_theta1_inc,no_of_solutions,old_theta[0]);
		   }
		   else{
			    // std::cout <<"root_pointer nie byl null" << std::endl;
			      if(no_of_solutions == SINGLE_SOLUTION)
					break;
			      previous = NULL;
			      node_pointer = root_pointer;
			      while( node_pointer != NULL)   {
					 previous = node_pointer;
					 node_pointer = node_pointer->NextTheta;
			       } /* end while( node_pointer != NULL) */
			      previous->NextTheta = Create1(x2,p,r,t,result,max_theta1_inc,no_of_solutions,old_theta[0]);
		  } /* end else od: if(root_pointer == NULL) */

		  switch (*result)
		   {
		    case OUT_OF_MEMORY: return(NULL);
		    case OUT_OF_RANGE:
		    case INVALID_ROOT_X:
		    case ONE_NODE:
		    case TWO_NODES: break;
		    default : *result = UNKNOWN_ERROR_THETA1; return(NULL);
		   } /* end switch (*result) */

		   if(root_pointer == NULL)
		     { *result = INVALID_ROOTS_THETA1; return(NULL); }
		   break;

     case TRIVIAL_EQUATION:
			  /* tu lepiej zostawic poprzednia wartosc
			     THETA1  - aktualnie wstawiamy atan2(u0[X],u0[Y])
			   temp1 = sqrt(u0[X]*u0[X] + u0[Y]*u0[Y]);
			   temp2 = u0[X]/temp1;
			   temp1 = u0[Y]/temp1;
			   */
			   {
			    if((root_pointer = Add_Theta(old_theta[0],
						cos(old_theta[0]),sin(old_theta[0]))) == NULL)
				    { *result = OUT_OF_MEMORY; return(NULL); }
			    else break;
			   }
     default :
			  *result = UNKNOWN_ERROR_THETA1; return(NULL);
    }  /* end switch(res) */

/* stworzenie poddrzew rozwiaza`n dla kazdego dopuszczalnego THETA1 */
  previous = NULL;
  node_pointer = root_pointer;
  while( node_pointer != NULL)
   {
     temp_pointer = Theta_4(node_pointer,u0,v0,u6,v6,q0,q6,
			    radius_2,result,old_theta,
			    interpolation_period,no_of_solutions);
     if(temp_pointer != NULL)
       { /* istnieje poddrzewo rozwiaza`n */
	 node_pointer->NextAngle = temp_pointer;
	 if( no_of_solutions == SINGLE_SOLUTION )
	   {
	     /* skasowac zbedne wezly warstwy THETA1 */
		temp_pointer = node_pointer->NextTheta;
		while( temp_pointer != NULL)
		  {
		    tmp_ptr = temp_pointer->NextTheta;
		    free(temp_pointer);
		    temp_pointer = tmp_ptr;
		  } /* end while( temp_pointer != NULL) */
		node_pointer->NextTheta = NULL;
	     /* przerwac obliczenia */
		break;
	   } /* end: if( no_of_solutions == SINGLE_SOLUTION ) */
	 previous = node_pointer;
	 node_pointer = node_pointer->NextTheta;
	}
     else
       {/* nie istnieje poddrzewo rozwiaza`n */
	 tmp_ptr = node_pointer->NextTheta;
	 if(previous != NULL)
	   previous->NextTheta = tmp_ptr;
	 else
	   root_pointer = tmp_ptr;
	 free(node_pointer);
	 node_pointer = tmp_ptr;
       }
   } /* end while( node_pointer != NULL) */

  if(root_pointer != NULL)
    *result = OK;

  return(root_pointer);

} /* Koniec Theta_1() */



/***************************************************************************
*                                                                          *
*                              Theta_2                                     *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Theta_2 oblicza kat THETA2 w trakcie rozwiazywania odwrotnego
  zagadnienia kinematycznego dla robota IRp-6. Tworzy wezly THETA2
  poddrzewa rozwiaza`n oraz wywoluje funkcje obliczajace pozostale
  THETA dla dopuszczalnych THETA2. Funkcje te tworza adekwatne podpoddrzewa
  rozwiaza`n.


WYWOlANIE:

   ptr = Theta_2(theta1_pointer, theta4_pointer, u0, v0, q0, u6, q6, v6,
		 radius_2, result, old_theta, interpolation_period,
		 no_of_solutions);


DANE WEJSCIOWE:

  THETA_NODE *theta1_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA1, dla ktorego obliczane sa
			       wartosci kata THETA2
  THETA_NODE *theta4_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA4, dla ktorego obliczane sa
			       wartosci kata THETA2

  double v0[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem bazowego ukladu odniesienia
  double u0[3] - wektor (zwiazany) srodka okregu, na ktorym moze
		 znajdowac sie kolnierz, wzgledem bazowego ukladu
		 odniesienia
  double u6[3] - wektor (zwiazany) srodka okregu, na ktorym moze
		 znajdowac sie kolnierz, wzgledem ukladu
		 wspolrzednych zwiazanego z kolnierzem
  double q0[3] - polozenie TCP w ukladzie bazowym
  double q6[3] - polozenie TCP w ukladzie kolnierza
  double v6[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem ukladu wspolrzednych zwiazanego z kolnierzem

  double radius_2 - kwadrat promienia okregu, na ktorym moze
		    znajdowac sie srodek kolnierza

  double old_theta[5] - poprzednia wartosc wejsciowego wektora
			wspolrzednych wewnetrznych

  double interpolation_period   - okres makrointerpolacji w [ms]

  int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				  byc obliczone wszystkie rozwiazania, czy
				  tez jedynie to lezace najblizej
				  poprzedniego


DANE WYJSCIOWE:

  Wartosc funkcji:
   THETA_NODE *ptr - wskaznik na wezel poddrzewa rozwiaza`n zawierajacy
		      pierwsza dopuszczalna wartosc THETA4 lub NULL,
		      gdy jej brak.

    int16_t *result - czy wynik dzialania poprawny

    *result = OK - stworzono drzewo rozwiaza`n.
    *result = THETA2_OUT_OF_RANGE
    *result = INCONSISTENT_DATA_THETA2
    *result = UNKNOWN_ERROR_THETA2
    *result = OUT_OF_MEMORY
    *result = INVALID_FLANGE
    *result = bledy zwracane przez funkcje wywolywane przez Theta_2


ZMIENNE ZEWNeTRZNE:

  Poddrzewo rozwiaza`n odwrotnego zagadnienia kinematycznego dla robota
  IRp-6. Znajduje sie na stercie (ang. heap). Dostep poprzez wskazniki
  lokalne.

---------------------------------------------------------------------------*/

THETA_NODE* model_5dof::Theta_2(THETA_NODE *theta1_pointer, THETA_NODE *theta4_pointer,
	    double u0[3], double v0[3], double q0[3], double u6[3],
	    double q6[3], double v6[3], double radius_2,
	    int16_t *result, double old_theta[5],
	    double interpolation_period, int16_t no_of_solutions)
{
 double theta_x1;   /* zmiena pomocnicza przeznaczona na przechowywanie
		       pierwszego rozwiazania rownania E cos + F sin - G = 0 */
 double theta_x2;   /* zmiena pomocnicza przeznaczona na przechowywanie
		       drugiego rozwiazania rownania E cos + F sin - G = 0 */
 double o06_prim[3];/* polozenia srodka kolnierza wzgledem ukladu
			 bazowego */
 double o06_bis[3]; /* polozenia srodka kolnierza wzgledem ukladu
			 bazowego */
 int16_t rrr;        /* zmienna pomocnicza przeznaczona na przechowywanie
			 liczby rozwiaza`n dla kolnierza */
 int16_t res;        /* zmienna pomocnicza przeznaczona na przechowywanie
			 rodzaju rozwiaza`nia rownania kwadratowego */
 double e1;         /* parametr E pierwszego zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double f1;         /* parametr F pierwszego zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double g1;         /* parametr G pierwszego zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double e2;         /* parametr E drugiego zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double f2;         /* parametr F drugiego zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double g2;         /* parametr G drugiego zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double max_theta2_inc;     /* maksymalny dopuszczalny przyrost ka`ta
				 THETA w 1 kroku makro-interpolacji */

 THETA_NODE *root_pointer;  /* wskaznik na korze`n w poddrzewie
				 rozwiaza`n */
 THETA_NODE *tmp_ptr;       /* wskaznik na wezel w drzewie
				 rozwiaza`n  */
 THETA_NODE *last_node;     /* wskaznik na ostatni wezel warstwy */


/* Poczatek Theta_2() */
//// std::cout << "Jestem w Theta_2" << std::endl;

/* inicjalizacja wskaznikow */
   root_pointer = NULL;
   last_node = NULL;
   tmp_ptr = NULL;

    *result = OK;

/* obliczenie maksymalnego dopuszczalnego przyrostu kata THETA w jednym
   kroku makro-interpolacji */
   max_theta2_inc = interpolation_period * MAX_THETA2_VELOCITY / 1000.0;

/* obliczenie wspolrzednych srodka kolnierza */
  rrr = Flange(u0, v0, u6, radius_2, theta1_pointer, theta4_pointer,
	       o06_prim, o06_bis);
  if(rrr == NO_SOLUTION)
    { *result = INVALID_FLANGE; return(NULL); }


 /* obliczenie parametrow rownania wyjsciowgo:   Ecos + Fsin - G = 0    */
    e1 = o06_prim[X]*C1 + o06_prim[Y]*S1 - d6*C4;
    f1 = d1 - d6*S4 - o06_prim[Z];
    g1 = (e1*e1 +f1*f1 +a2*a2 - a3*a3)/(2.0*a2);

   res = Ecos_Fsin_G(e1, f1, g1, &theta_x1, &theta_x2,
		     LOWER_THETA2_LIMIT, UPPER_THETA2_LIMIT, max_theta2_inc,
		     old_theta[1], no_of_solutions);

  /* stworzenie warstwy THETA2 w drzewie rozwiaza`n */
  switch(res)
   {
     case NO_SOLUTION:
	  if(rrr == ONE_SOLUTION)
	    {*result = THETA2_OUT_OF_RANGE; return(NULL);}
	  else break;

     case INCONSISTENT_DATA:
	  if(rrr == ONE_SOLUTION)
	    {*result = INCONSISTENT_DATA_THETA2; return(NULL);}
	  else break;

     case ONE_SOLUTION:
     case TRIVIAL_EQUATION:
	  if( (root_pointer = Add_Theta(theta_x1,cos(theta_x1),sin(theta_x1)))
	       == NULL)
	    { *result = OUT_OF_MEMORY; return(NULL); }
	  else
	    {
	      /* utworzenie poddrzewa rozwiaza`n */
	      if( (root_pointer->NextAngle =
		   Theta_3(theta1_pointer,root_pointer,theta4_pointer,e1,f1,q0,q6,v0,v6,result,
		     old_theta,interpolation_period,no_of_solutions))
		    == NULL)
		{ free(root_pointer); root_pointer = NULL; }
	      last_node = root_pointer;
	      break;
	    }

     case TWO_SOLUTIONS:

	  /* pierwsze rozwiazanie */
	  if( (root_pointer = Add_Theta(theta_x1,cos(theta_x1),sin(theta_x1)))
	       == NULL)
	    { *result = OUT_OF_MEMORY; return(NULL); }
	  else
	    {
	      /* utworzenie poddrzewa rozwiaza`n */
	      if( (root_pointer->NextAngle =
		      Theta_3(theta1_pointer,root_pointer,theta4_pointer,e1,f1,q0,q6,v0,v6,result,
			     old_theta,interpolation_period,no_of_solutions))
		    == NULL)
		{ free(root_pointer); root_pointer = NULL; }
	      last_node = root_pointer;
	    }

	  if( (no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL))
	    {
	      /* drugie rozwiazanie */
	      if( (last_node = Add_Theta(theta_x2,cos(theta_x2),sin(theta_x2)))
		   == NULL)
		{ *result = OUT_OF_MEMORY; return(NULL); }
	      else
		{
		  /* utworzenie poddrzewa rozwiaza`n */
		  if( (last_node->NextAngle =
			  Theta_3(theta1_pointer,last_node,theta4_pointer,e1,f1,q0,q6,v0,v6,result,
				 old_theta,interpolation_period,no_of_solutions))
			== NULL)
		    { free(last_node); last_node = root_pointer; }
		  if(root_pointer == NULL)
		    root_pointer = last_node;
		  else
		    root_pointer->NextTheta = last_node;
		  break;
		}
	    }  /* end: if( (no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL)) */

     default :
	  *result = UNKNOWN_ERROR_THETA2; return(NULL);
    }  /* end switch(res) */



 if((no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL))
  if(rrr == TWO_SOLUTIONS)
   {  /* rrr == TWO_SOLUTIONS */

     /* parametry rownania wyjsciowego:   Ecos + Fsin - G = 0    */
	e2 = o06_bis[X]*C1 + o06_bis[Y]*S1 - d6*C4;
	f2 = d1 - d6*S4 - o06_bis[Z];
	g2 = (e2*e2 +f2*f2 +a2*a2 - a3*a3)/(2.0*a2);

	res = Ecos_Fsin_G(e2, f2, g2, &theta_x1, &theta_x2,
		     LOWER_THETA2_LIMIT, UPPER_THETA2_LIMIT, max_theta2_inc,
		     old_theta[1], no_of_solutions);

      /* kontynuacja tworzenia warstwy THETA2 w drzewie rozwiaza`n */

     switch(res)
      {
	case NO_SOLUTION:
	     if(root_pointer == NULL)
	       {*result = THETA2_OUT_OF_RANGE; return(NULL);}
	     else break;

	case INCONSISTENT_DATA:
	     if(root_pointer == NULL)
	       {*result = INCONSISTENT_DATA_THETA2; return(NULL);}
	     else break;

	case ONE_SOLUTION:
	case TRIVIAL_EQUATION:
	     if( (tmp_ptr = Add_Theta(theta_x1,cos(theta_x1),sin(theta_x1)))
		  == NULL)
	       { *result = OUT_OF_MEMORY; return(NULL); }
	  else
	    {
	      /* utworzenie poddrzewa rozwiaza`n */
	      if( (tmp_ptr->NextAngle =
		      Theta_3(theta1_pointer,tmp_ptr,theta4_pointer,e2,f2,q0,q6,v0,v6,result,
			     old_theta,interpolation_period,no_of_solutions))
		    == NULL)
		{ free(tmp_ptr); tmp_ptr = NULL; }
	      if(last_node == NULL)
		 root_pointer = tmp_ptr;
	      else
		last_node->NextTheta = tmp_ptr;
	      break;
	    }

	case TWO_SOLUTIONS:

	     /* pierwsze rozwiazanie */
	     if( (tmp_ptr = Add_Theta(theta_x1,cos(theta_x1),sin(theta_x1)))
		  == NULL)
	       { *result = OUT_OF_MEMORY; return(NULL); }
	     else
	       {
		 /* utworzenie poddrzewa rozwiaza`n */
		 if( (tmp_ptr->NextAngle =
			 Theta_3(theta1_pointer,tmp_ptr,theta4_pointer,e2,f2,q0,q6,v0,v6,result,
				old_theta,interpolation_period,no_of_solutions))
		       == NULL)
		   { free(tmp_ptr); tmp_ptr = NULL; }
		 if(last_node == NULL)
		    root_pointer = tmp_ptr;
		 else
		   last_node->NextTheta = tmp_ptr;
		 last_node = tmp_ptr;
	       }

	  if( (no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL))
	    {
	     /* drugie rozwiazanie */
	     if( (tmp_ptr =
		  Add_Theta(theta_x2,cos(theta_x2),sin(theta_x2))) == NULL)
	       { *result = OUT_OF_MEMORY; return(NULL); }
	      else
		{
		  /* utworzenie poddrzewa rozwiaza`n */
		  if( (tmp_ptr->NextAngle =
			  Theta_3(theta1_pointer,tmp_ptr,theta4_pointer,e2,f2,q0,q6,v0,v6,result,
				 old_theta,interpolation_period,no_of_solutions))
			== NULL)
		    { free(tmp_ptr); tmp_ptr = last_node; }
		  if(root_pointer == NULL)
		    root_pointer = tmp_ptr;
		  else
		    last_node->NextTheta = tmp_ptr;
		  break;
		}
	    }  /* end: if( (no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL)) */

	default :
	     *result = UNKNOWN_ERROR_THETA2; return(NULL);
       }  /* end switch(res) */

   }  /* end: if(rrr == TWO_SOLUTIONS) */
 /* end:  if((no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL)) */

 return(root_pointer);

} /* Koniec Theta_2() */



/***************************************************************************
*                                                                          *
*                              Theta_3                                     *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Theta_3 oblicza kat THETA3 w trakcie rozwiazywania odwrotnego
  zagadnienia kinematycznego dla robota IRp-6. Tworzy wezel THETA3
  poddrzewa rozwiaza`n oraz wywoluje funkcje obliczajace pozostale
  THETA dla dopuszczalnych THETA3. Funkcje te tworza adekwatne podpoddrzewa
  rozwiaza`n.


WYWOlANIE:

   ptr = Theta_3(theta1_pointer, theta2_pointer, theta4_pointer,
		 e, f, q0, q6, v0, v6, result, old_theta,
		 interpolation_period, no_of_solutions);


DANE WEJSCIOWE:

  THETA_NODE *theta1_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA1, dla ktorego obliczane sa
			       wartosci kata THETA2
  THETA_NODE *theta2_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA2, dla ktorego obliczane sa
			       wartosci kata THETA3
  THETA_NODE *theta4_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA4, dla ktorego obliczane sa
			       wartosci kata THETA3

  double e     - parametr rownania: E c2  +  F s2 = G
  double f     - parametr rownania: E c2  +  F s2 = G

  double q0[3] - polozenie TCP w ukladzie bazowym
  double q6[3] - polozenie TCP w ukladzie kolnierza
  double v0[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem bazowego ukladu odniesienia
  double v6[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem ukladu wspolrzednych zwiazanego z kolnierzem

  double old_theta[5] - poprzednia wartosc wejsciowego wektora
			wspolrzednych wewnetrznych

  double interpolation_period   - okres makrointerpolacji w [ms]

  int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				  byc obliczone wszystkie rozwiazania, czy
				  tez jedynie to lezace najblizej
				  poprzedniego


DANE WYJSCIOWE:

  Wartosc funkcji:
   THETA_NODE *ptr - wskaznik na wezel poddrzewa rozwiaza`n zawierajacy
		      dopuszczalna wartosc THETA3 lub NULL, gdy jej brak.

    int16_t *result - czy wynik dzialania poprawny

    *result = OK - stworzono drzewo rozwiaza`n.
    *result = OUT_OF_MEMORY
    *result = ATAN_THETA3_ERROR
    *result = THETA3_OUT_OF_RANGE
    *result = bledy zwracane przez funkcje wywolywane przez Theta_3


ZMIENNE ZEWNeTRZNE:

  Poddrzewo rozwiaza`n odwrotnego zagadnienia kinematycznego dla robota
  IRp-6. Znajduje sie na stercie (ang. heap). Dostep poprzez wskazniki
  lokalne.

---------------------------------------------------------------------------*/

/* o tyle moze byc przekroczone ograniczenie narzucone na THETA ->
   spowodowane kumulacja bledow numerycznych */
#define OFFSET 1.0e-4

THETA_NODE* model_5dof::Theta_3(THETA_NODE *theta1_pointer,
		    THETA_NODE *theta2_pointer,
		    THETA_NODE *theta4_pointer,
		    double e, double f,
		    double q0[3], double q6[3],
		    double v0[3], double v6[3],
		    int16_t *result, double old_theta[5],
		    double interpolation_period,
		    int16_t no_of_solutions)
{
   double eee;                /* zmienna pomocnicza: e - a2*cos(THETA2) */
   double fff;                /* zmienna pomocnicza: f - a2*sin(THETA2) */
   double theta3;             /* wartosc kata THETA3 */
   double max_theta3_inc;     /* maksymalny dopuszczalny przyrost ka`ta
				 THETA w 1 kroku makro-interpolacji */
   THETA_NODE *root_pointer;  /* wskaznik na korze`n w poddrzewie
				 rozwiaza`n */

/* Poczatek Theta_3() */
// // std::cout << "Jestem w Theta_3" << std::endl;
/* inicjalizacja wskaznika */
   root_pointer = NULL;

/* obliczenie maksymalnego dopuszczalnego przyrostu kata THETA w jednym
   kroku makro-interpolacji */
   max_theta3_inc = interpolation_period * MAX_THETA3_VELOCITY / 1000.0;

  eee = e - a2*C2;
  fff = f - a2*S2;
  if( (eee < EPS) && (eee > -EPS))  /* eee == 0 */
     /* eee == 0  => cos(theta3) == 0  => theta3 == -90 lub +90,
	natomiast -25 < theta3 < 40  => blad */
     { *result = ATAN_THETA3_ERROR; return(NULL); }
  else
    {   /*  eee != 0  */
      theta3 = atan2(fff,eee);
      if( (theta3 > UPPER_THETA3_LIMIT) || (theta3 < LOWER_THETA3_LIMIT) )
	{
	  if( (theta3 > UPPER_THETA3_LIMIT+OFFSET) || (theta3 < LOWER_THETA3_LIMIT-OFFSET) )
	    {
	       *result = THETA3_OUT_OF_RANGE;
	       return(NULL);
	    }
	  else
	    if(theta3 < UPPER_THETA3_LIMIT)
	       theta3 = LOWER_THETA3_LIMIT;
	    else
	       theta3 = UPPER_THETA3_LIMIT;
	}
      if( (no_of_solutions == SINGLE_SOLUTION)  &&
	  (fabs(theta3 - old_theta[2]) > max_theta3_inc))
	{
	  *result = THETA3_OUT_OF_RANGE;
	  return(NULL);
	}
      if((root_pointer = Add_Theta(theta3,eee/a3,fff/a3)) == NULL)
	{
	  *result = OUT_OF_MEMORY;
	  return(NULL);
	}

      /* stworzenie poddrzew rozwiaza`n dla dopuszczalnego THETA3 */

      root_pointer->NextAngle = Theta_5(theta1_pointer,theta2_pointer,
		       root_pointer,theta4_pointer,q0,q6,v0,v6,
				  result,old_theta,
		       interpolation_period,no_of_solutions);
      if(root_pointer->NextAngle != NULL)
	{ /* istnieje poddrzewo rozwiaza`n */
	  *result = OK;
	  return(root_pointer);
	}
      else
	{/* nie istnieje poddrzewo rozwiaza`n */
	  free(root_pointer);
	  return(NULL);
	}
     }   /*  end:  eee != 0  */

} /* Koniec Theta_3() */



/***************************************************************************
*                                                                          *
*                              Theta_4                                     *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Theta_4 oblicza kat THETA4 w trakcie rozwiazywania odwrotnego
  zagadnienia kinematycznego dla robota IRp-6. Tworzy wezly THETA4
  poddrzewa rozwiaza`n oraz wywoluje funkcje obliczajace pozostale
  THETA dla dopuszczalnych THETA4. Funkcje te tworza adekwatne podpoddrzewa
  rozwiaza`n.


WYWOlANIE:

   ptr = Theta_4(theta1_pointer, u0,  v0,  u6,  v6, q0,  q6,  radius_2,
		 result, old_theta,interpolation_period, no_of_solutions);


DANE WEJSCIOWE:

  THETA_NODE *theta1_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA1, dla ktorego obliczane sa
			       wartosci kata THETA4
  double v0[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem bazowego ukladu odniesienia
  double v6[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem ukladu wspolrzednych zwiazanego z kolnierzem
  double u0[3] - wektor (zwiazany) srodka okregu, na ktorym moze
		 znajdowac sie kolnierz, wzgledem bazowego ukladu
		 odniesienia
  double u6[3] - wektor (zwiazany) srodka okregu, na ktorym moze
		 znajdowac sie kolnierz, wzgledem ukladu wspolrzednych
		 zwiazanego z kolnierzem
  double q0[3] - polozenie TCP w ukladzie bazowym
  double q6[3] - polozenie TCP w ukladzie kolnierza

  double old_theta[5] - poprzednia wartosc wejsciowego wektora
			wspolrzednych wewnetrznych

  double interpolation_period   - okres makrointerpolacji w [ms]

  int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				  byc obliczone wszystkie rozwiazania, czy
				  tez jedynie to lezace najblizej
				  poprzedniego


DANE WYJSCIOWE:

  Wartosc funkcji:
   THETA_NODE *ptr - wskaznik na wezel poddrzewa rozwiaza`n zawierajacy
		      pierwsza dopuszczalna wartosc THETA4 lub NULL,
		      gdy jej brak.

    int16_t *result - czy wynik dzialania poprawny

    *result = OK - stworzono drzewo rozwiaza`n.
    *result = THETA4_OUT_OF_RANGE
    *result = INCONSISTENT_DATA_THETA4
    *result = UNKNOWN_ERROR_THETA4
    *result = OUT_OF_MEMORY
    *result = bledy zwracane przez funkcje wywolywane przez Theta_4


ZMIENNE ZEWNeTRZNE:

  Poddrzewo rozwiaza`n odwrotnego zagadnienia kinematycznego dla robota
  IRp-6. Znajduje sie na stercie (ang. heap). Dostep poprzez wskazniki
  lokalne.

---------------------------------------------------------------------------*/

THETA_NODE* model_5dof::Theta_4(THETA_NODE *theta1_pointer,
		    double u0[3], double v0[3], double u6[3], double v6[3],
		    double q0[3], double q6[3], double radius_2,
		    int16_t *result, double old_theta[5],
		    double interpolation_period,
		    int16_t no_of_solutions)
{
 double theta_x1;   /* zmiena pomocnicza przeznaczona na przechowywanie
		       pierwszego rozwiazania rownania E cos + F sin - G = 0 */
 double theta_x2;   /* zmiena pomocnicza przeznaczona na przechowywanie
		       drugiego rozwiazania rownania E cos + F sin - G = 0 */
 int16_t res; /* zmienna pomocnicza przeznaczona na przechowywanie
			 rodzaju rozwiaza`nia rownania kwadratowego */
 double e;           /* parametr E zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double f;           /* parametr F zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double g;           /* parametr G zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double max_theta4_inc;     /* maksymalny dopuszczalny przyrost ka`ta
				 THETA w 1 kroku makro-interpolacji */

 THETA_NODE *root_pointer;  /* wskaznik na korze`n w poddrzewie
				 rozwiaza`n */
 THETA_NODE *tmp_ptr;       /* wskaznik na wezel w poddrzewie
				 rozwiaza`n */

/* Poczatek Theta_4() */

// 	// std::cout << "Jestem w Theta_4" << std::endl;
/* inicjalizacja wskaznika */
   root_pointer = NULL;

    *result = OK;

/* obliczenie maksymalnego dopuszczalnego przyrostu kata THETA w jednym
   kroku makro-interpolacji */
   max_theta4_inc = interpolation_period * MAX_THETA4_VELOCITY / 1000.0;
 /* parametry rownania */
 /* rownanie wyjsciowe:   Ecos + Fsin - G = 0    */
    e =  v0[X]*C1 + v0[Y]*S1;
    f = -v0[Z];
    g =  v6[Z];

    res = Ecos_Fsin_G(e, f, g, &theta_x1, &theta_x2,
		     LOWER_THETA4_LIMIT, UPPER_THETA4_LIMIT, max_theta4_inc,
		     old_theta[3], no_of_solutions);

  /* stworzenie warstwy THETA4 w drzewie rozwiaza`n */

  switch(res)
   {
     case NO_SOLUTION:
	  *result = THETA4_OUT_OF_RANGE; return(NULL);

     case INCONSISTENT_DATA:
	  *result = INCONSISTENT_DATA_THETA4; return(NULL);

     case ONE_SOLUTION:
     case TRIVIAL_EQUATION:
	  if( (root_pointer = Add_Theta(theta_x1,cos(theta_x1),sin(theta_x1)))
	       == NULL)
	    { *result = OUT_OF_MEMORY; return(NULL); }
	  else
	    {
	      root_pointer->NextAngle = Theta_2(theta1_pointer,root_pointer,
				u0,v0,q0,u6,q6,v6,radius_2,result,old_theta,
				interpolation_period,no_of_solutions);
	      if(root_pointer->NextAngle == NULL)
		{ /* nie istnieje poddrzewo rozwiaza`n */
		  free(root_pointer);  root_pointer = NULL;
		}
	      return(root_pointer);
	    }  /* end: else */

     case TWO_SOLUTIONS:

	  /* pierwsze rozwiazanie */
	  if( (root_pointer = Add_Theta(theta_x1,cos(theta_x1),sin(theta_x1)))
	       == NULL)
	    { *result = OUT_OF_MEMORY; return(NULL); }
	  else
	    {
	      root_pointer->NextAngle = Theta_2(theta1_pointer,root_pointer,
				u0,v0,q0,u6,q6,v6,radius_2,result,old_theta,
				interpolation_period,no_of_solutions);
	      if(root_pointer->NextAngle == NULL)
		{ /* nie istnieje poddrzewo rozwiaza`n */
		  free(root_pointer);  root_pointer = NULL;
		}
	    }  /* end: else */

	   if( (no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL))
	    {
	      /* drugie rozwiazanie */
	       if( (tmp_ptr = Add_Theta(theta_x2,cos(theta_x2),sin(theta_x2))) == NULL)
		 { *result = OUT_OF_MEMORY; return(NULL); }
	       else
		 {
		   tmp_ptr->NextAngle = Theta_2(theta1_pointer,tmp_ptr,
				     u0,v0,q0,u6,q6,v6,radius_2,result,old_theta,
				     interpolation_period,no_of_solutions);
		   if(tmp_ptr->NextAngle == NULL)
		     { /* nie istnieje poddrzewo rozwiaza`n */
		       free(tmp_ptr);
		     }
		   else
		     if( root_pointer == NULL )
		       root_pointer = tmp_ptr;
		     else
		       root_pointer->NextAngle = tmp_ptr;
		 }  /* end: else */
	    } /* end: if( (no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL)) */
	   return(root_pointer);

     default :
	  *result = UNKNOWN_ERROR_THETA4; return(NULL);
    }  /* end switch(res) */

} /* Koniec Theta_4() */



/***************************************************************************
*                                                                          *
*                              Theta_5                                     *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Theta_5 oblicza kat THETA5 w trakcie rozwiazywania odwrotnego
  zagadnienia kinematycznego dla robota IRp-6. Tworzy wezly THETA5
  poddrzewa rozwiaza`n. Dla danego zestawu katow THETA1, THETA2,
  THETA3, THETA4 moze istniec od 0 do 2 wartosci kata THETA5.


WYWOlANIE:

 ptr = Theta_5(theta1_pointer, theta2_pointer, theta3_pointer, theta4_pointer,
	       q0,  q6, v0, v6, result, old_theta, interpolation_period,
		no_of_solutions);


DANE WEJSCIOWE:

  THETA_NODE *theta1_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA1, dla ktorego obliczane sa
			       wartosci kata THETA2
  THETA_NODE *theta2_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA2, dla ktorego obliczane sa
			       wartosci kata THETA5
  THETA_NODE *theta3_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA3, dla ktorego obliczane sa
			       wartosci kata THETA5
  THETA_NODE *theta4_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA4, dla ktorego obliczane sa
			       wartosci kata THETA5

  double q0[3] - polozenie TCP w ukladzie bazowym
  double q6[3] - polozenie TCP w ukladzie kolnierza
  double v0[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem bazowego ukladu odniesienia
  double v6[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem ukladu wspolrzednych zwiazanego z kolnierzem

  double old_theta[5] - poprzednia wartosc wejsciowego wektora
			  wspolrzednych wewnetrznych

  double interpolation_period   - okres makrointerpolacji w [ms]

  int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				  byc obliczone wszystkie rozwiazania, czy
				  tez jedynie to lezace najblizej
				  poprzedniego


DANE WYJSCIOWE:

  Wartosc funkcji:
   THETA_NODE *ptr - wskaznik na wezel poddrzewa rozwiaza`n zawierajacy
		      pierwsza dopuszczalna wartosc THETA4 lub NULL,
		      gdy jej brak.

    int16_t *result - czy wynik dzialania poprawny

    *result = OK - stworzono drzewo rozwiaza`n.
    *result = INCONSISTENT_DATA_THETA5
    *result = THETA5_OUT_OF_RANGE
    *result = THETA5_EQUATION_VIOLATION
    *result = UNKNOWN_ERROR_THETA5
    *result = OUT_OF_MEMORY


ZMIENNE ZEWNeTRZNE:

  Poddrzewo rozwiaza`n odwrotnego zagadnienia kinematycznego dla robota
  IRp-6. Znajduje sie na stercie (ang. heap). Dostep poprzez wskazniki
  lokalne.

---------------------------------------------------------------------------*/

#define EPS5 1.0e-6

THETA_NODE* model_5dof::Theta_5(THETA_NODE *theta1_pointer,
		    THETA_NODE *theta2_pointer,
		    THETA_NODE *theta3_pointer,
		    THETA_NODE *theta4_pointer,
		    double q0[3], double q6[3],
		    double v0[3], double v6[3],
		    int16_t *result, double old_theta[5],
		    double interpolation_period,
		    int16_t no_of_solutions)

{
 double theta_x1;   /* zmiena pomocnicza przeznaczona na przechowywanie
		       pierwszego rozwiazania rownania E cos + F sin - G = 0 */
 double theta_x2;   /* zmiena pomocnicza przeznaczona na przechowywanie
		       drugiego rozwiazania rownania E cos + F sin - G = 0 */
 int16_t res; /* zmienna pomocnicza przeznaczona na przechowywanie
		       rodzaju rozwiaza`nia rownania E cos + F sin - G = 0 */
 double e;           /* parametr E zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double f;           /* parametr F zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double g;           /* parametr G zestawu parametrow rownania:
			  E cos + F sin - G = 0 */
 double s5;          /* sin(THETA5) */
 double c5;          /* cos(THETA5) */

 int16_t special_case;  /* gdy C4 == 0 && S4 > 0   special_case =  1
		       gdy C4 == 0 && S4 < 0   special_case = -1
		       gdy C4 != 0             special_case =  0 */

 double max_theta5_inc;     /* maksymalny dopuszczalny przyrost ka`ta
			       THETA w 1 kroku makro-interpolacji */

 THETA_NODE *root_pointer;  /* wskaznik na korze`n w poddrzewie
			       rozwiaza`n */
 THETA_NODE *tmp_ptr;       /* pomocniczy wska`xnik na wezel
			       drzewa rozwiaza`n */

/* Poczatek Theta_5() */

// // std::cout << "Jestem w Theta_5" << std::endl;
/* inicjalizacja wskaznika */
   root_pointer = NULL;

   special_case = 0;

/* obliczenie maksymalnego dopuszczalnego przyrostu kata THETA5 w jednym
   kroku makro-interpolacji */
   max_theta5_inc = interpolation_period * MAX_THETA5_VELOCITY / 1000.0;

/* obliczenie wspolczynnikow rownania E cos + F sin - G = 0 */

 /* parametry rownania */
 /* rownanie wyjsciowe:   Ecos + Fsin - G = 0    */

  if( (C4 < EPS5) && (C4 > -EPS5) )
    { /* C4 == 0 */
      g = q0[X] - C1*(d6*C4 + a3*C3 + a2*C2);
      if( S4 > 0 )  /* S4 == 1 */
	{
	  special_case = 1;
	  e = C1*q6[X] + S1*q6[Y];
			 f = S1*q6[X] - C1*q6[Y];
	  if( (e < EPS5) && (e > -EPS5)  &&  (f < EPS5) && (f > -EPS5) )
		{  /* e == 0 && f == 0 */
		  e =  C1*v6[X] + S1*v6[Y];
		  f =  S1*v6[X] - C1*v6[Y];
		  g =  v0[X];
		}
	 }
       else  /* S4 == -1 */
	 {
	   special_case = -1;
			  e = -C1*q6[X] + S1*q6[Y];
		  f = S1*q6[X] + C1*q6[Y];
	   if( (e < EPS5) && (e > -EPS5)  &&  (f < EPS5) && (f > -EPS5) )
		{  /* e == 0 && f == 0 */
		  e =  -C1*v6[X] + S1*v6[Y];
		  f =  S1*v6[X] + C1*v6[Y];
		  g =  v0[X];
		}
	 }
    } /* end: C4 == 0 */
  else
    { /* C4 != 0 */
      e =  q6[X]*C4;
      f = -q6[Y]*C4;
      g =  q6[Z]*S4 - d1 + d6*S4 + a3*S3 + a2*S2 + q0[Z];
      if( (e < EPS5) && (e > -EPS5)  &&  (f < EPS5) && (f > -EPS5) )
	{  /* e == 0 && f == 0 */
	  e =  v6[X]*C4;
	  f = -v6[Y]*C4;
	  g =  v6[Z]*S4 + v0[Z];
	}
    } /* end: C4 != 0 */

   res = Ecos_Fsin_G(e, f, g, &theta_x1, &theta_x2,
		     LOWER_THETA5_LIMIT, UPPER_THETA5_LIMIT, max_theta5_inc,
		     old_theta[4], ALL_SOLUTIONS);
  /* Tutaj zadamy znalezienia wszystkich rozwiaza`n niezaleznie od no_of_solutions,
     poniewaz nie chcemy, aby przedwczesnie byly sprawdzone ograniczenia
     predkosciowe. Faktyczne sprawdzenie ogranicze`n
     predkosciowych zostanie zrobione przez Check_Theta5 */

  /* stworzenie warstwy THETA5 w drzewie rozwiaza`n */

  switch(res)
   {
     case NO_SOLUTION:
	  *result = THETA5_OUT_OF_RANGE; return(NULL);
     case INCONSISTENT_DATA:
	  *result = INCONSISTENT_DATA_THETA5; return(NULL);
     case ONE_SOLUTION:
     case TRIVIAL_EQUATION:
	  s5 = sin(theta_x1);
	  c5 = cos(theta_x1);
	  if( Check_Theta5(&theta_x1, c5, s5, q0, q6, v0, v6,
		 theta1_pointer, theta2_pointer, theta3_pointer, theta4_pointer,
		 no_of_solutions, max_theta5_inc, old_theta[4])
	       == REJECT)
	    { *result = THETA5_EQUATION_VIOLATION; return(NULL); }
	  if( (root_pointer = Add_Theta(theta_x1,c5,s5)) == NULL)
	    { *result = OUT_OF_MEMORY; return(NULL); }
	  else
	    { *result = OK;  return(root_pointer);}
     case TWO_SOLUTIONS:

	  /* pierwsze rozwiazanie */
	  s5 = sin(theta_x1);
	  c5 = cos(theta_x1);
	  if( Check_Theta5(&theta_x1, c5, s5, q0, q6, v0, v6,
		 theta1_pointer, theta2_pointer, theta3_pointer, theta4_pointer,
		 no_of_solutions, max_theta5_inc, old_theta[4])
	       == ACCEPT)
	    if( (root_pointer = Add_Theta(theta_x1,c5,s5)) == NULL)
	      { *result = OUT_OF_MEMORY; return(NULL); }

	  if( (no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL))
	    {
	      /* drugie rozwiazanie */
	      s5 = sin(theta_x2);
	      c5 = cos(theta_x2);
	      if(Check_Theta5(&theta_x2, c5, s5, q0, q6, v0, v6,
		   theta1_pointer, theta2_pointer, theta3_pointer, theta4_pointer,
		   no_of_solutions, max_theta5_inc, old_theta[4])
		 == ACCEPT)
		{
		  if( (tmp_ptr = Add_Theta(theta_x2,c5,s5)) == NULL)
		    { *result = OUT_OF_MEMORY; return(NULL); }
		  if(root_pointer == NULL)
		    root_pointer = tmp_ptr;
		  else
		    root_pointer->NextTheta = tmp_ptr;
		}  /* end: Check_Theta5() ... */
	    } /* end: if( (no_of_solutions == ALL_SOLUTIONS) || (root_pointer == NULL)) */
	  if(root_pointer != NULL)
	    *result = OK;
	  else
	    *result = THETA5_EQUATION_VIOLATION;
	  return(root_pointer);

     default :
	  *result = UNKNOWN_ERROR_THETA5; return(NULL);
    }  /* end switch(res) */

} /* Koniec Theta_5() */



/***************************************************************************
*                                                                          *
*                             QuadraticEquation                            *
*                                                                          *
****************************************************************************

FUNKCJA:

Funkcja QuadraticEquation rozwiazuje rownanie kwadratowe o postaci:
    a x^2 + b x + c = 0
 Jezeli a = 0, to rozwiazuje rownanie liniowe:
   b x + c = 0


WYWOlANIE:

   r = QuadraticEquation(a, b, c, x1, x2, delta);


DANE WEJSCIOWE:

   double a      - wspolczynniki przy czynniku kwadratowym wielomianu kwadratowego
   double b      - wspolczynniki przy czynniku liniowym wielomianu kwadratowego
   double c      - wyraz wolnym wielomianu kwadratowego
   double delta  - wartosc wyroznika rownania kwadratowego


DANE WYJSCIOWE:

   double *x1, *x2 - wska`xniki na pierwiastki rownania

   Wartosc funkcji:
    r = NO_SOLUTION       - a != 0  i delta < 0   --> x1,x2 - nieokreslone
    r = ONE_SOLUTION      - a != 0  i delta = 0   --> x2    - nieokreslone
    r = TWO_SOLUTIONS     - a != 0  i delta > 0
    r = LINEAR_SOLUTION   - a = 0   i b != 0      --> x2    - nieokreslone
    r = INCONSISTENT_DATA - a = 0, b = 0, c != 0  --> x1,x2 - nieokreslone
    r = TRIVIAL_EQUATION  - a = 0, b = 0, c = 0   --> x1,x2 - nieokreslone

UWAGA! Jezeli ktorys z otrzymanych pierwiastkow nieznacznie
       przekracza 1 lub -1, to jest modyfikowany, tak aby stal
       sie 1 lub -1 odpowiednio. Stala ROUND_OFF determinuje
       stopie`n dopuszczalnego nadmiaru.

---------------------------------------------------------------------------*/

#undef  EPS
#define EPS 1.0e-15
#define ROUND_OFF 1.0e-8

int16_t model_5dof::QuadraticEquation(double a, double b, double c,
				double *x1, double *x2, double delta)
{
  double sqrt_delta;    /* pierwiastek wyroznika rownania kwadratowego */
  double sgnb;          /* znak wspolczynnika B rownania kwadratowego */
  double q;             /* q = -(b + sgnb*sqrt_delta)/2 */

/* Poczatek QuadraticEquation() */
   if ((a > EPS) || (a < -EPS)) { /* a != 0  =>  rownanie kwadratowe */
	      if (delta < 0){
			   if((-delta/(4.0*a*c) < ROUND_OFF) || (-delta/(a*a) < ROUND_OFF))
				   /* zakladamy, ze delta byla ujemna ze wzgledu na blad
				      numeryczny */
				      sqrt_delta = 0.0;
				  else
				      return(NO_SOLUTION);
		 }  /* end: if (delta < 0) */
	      else
			sqrt_delta = sqrt(delta);


	     if (sqrt_delta < EPS){
			 *x1 = -b/(2*a);
			 /* z powodu kumulacji bledow numerycznych pierwiastek, ktory
			    w rzeczywistosci = 1 lub -1 moze nieznacznie przekraczac te
			    wartosci, wiec jest modyfikowany. Wartosci 1 i -1 sa istotne
			    ze wzgledu na to, ze pierwiastek rownania kwadratowego jest
			    jednoczesnie cosinusem odpowiedniego kata.  */
			 if( ((*x1 <  1.0 + ROUND_OFF) && (*x1 >  1.0)) ) *x1 =  1.0;
			 if( ((*x1 > -1.0 - ROUND_OFF) && (*x1 < -1.0)) ) *x1 = -1.0;
			 return(ONE_SOLUTION);
	     }

	     if (b < 0)
			sgnb = -1.0;
		else
			sgnb = 1.0;

		q = -(b + sgnb*sqrt_delta)/2 ;

	     if ((q < EPS) && (q > -EPS)){
			   /* z powodu kumulacji bledow numerycznych pierwiastek, ktory
			      w rzeczywistosci = 1 lub -1 moze nieznacznie przekraczac te
			      wartosci, wiec jest modyfikowany. Wartosci 1 i -1 sa istotne
			      ze wzgledu na to, ze pierwiastek rownania kwadratowego jest
			      jednoczesnie cosinusem odpowiedniego kata.  */
			   *x1 = (-b + sqrt_delta)/(2*a);
			    if( ((*x1 <  1.0 + ROUND_OFF) && (*x1 >  1.0)) ) *x1 =  1.0;
			    if( ((*x1 > -1.0 - ROUND_OFF) && (*x1 < -1.0)) ) *x1 = -1.0;
			   *x2 = (-b - sqrt_delta)/(2*a);
			    if( ((*x2 <  1.0 + ROUND_OFF) && (*x2 >  1.0)) ) *x2 =  1.0;
			    if( ((*x2 > -1.0 - ROUND_OFF) && (*x2 < -1.0)) ) *x2 = -1.0;
			   return(TWO_SOLUTIONS);
	     }
	     else{
			   /* z powodu kumulacji bledow numerycznych pierwiastek, ktory
			      w rzeczywistosci = 1 lub -1 moze nieznacznie przekraczac te
			      wartosci, wiec jest modyfikowany. Wartosci 1 i -1 sa istotne
			      ze wzgledu na to, ze pierwiastek rownania kwadratowego jest
			      jednoczesnie cosinusem odpowiedniego kata.  */
			   *x1 = q/a ;
			    if( ((*x1 <  1.0 + ROUND_OFF) && (*x1 >  1.0)) ) *x1 =  1.0;
			    if( ((*x1 > -1.0 - ROUND_OFF) && (*x1 < -1.0)) ) *x1 = -1.0;
			   *x2 = c/q ;
			    if( ((*x2 <  1.0 + ROUND_OFF) && (*x2 >  1.0)) ) *x2 =  1.0;
			    if( ((*x2 > -1.0 - ROUND_OFF) && (*x2 < -1.0)) ) *x2 = -1.0;
			   return(TWO_SOLUTIONS);
	       }
   } /* koniec rozwiazania rownania kwadratowego */
  else{ /* rownanie liniowe */
	      if ((b < EPS) && (b > -EPS)) /* b == 0 */
			if ((c < EPS) && (c > -EPS)) /* c == 0 */
	 				return(TRIVIAL_EQUATION);
			else /* c != 0 */
					return(INCONSISTENT_DATA);
	    /* b != 0 */
	    *x1 = -c/b;
	   /* z powodu kumulacji bledow numerycznych pierwiastek, ktory
	      w rzeczywistosci = 1 lub -1 moze nieznacznie przekraczac te
	      wartosci, wiec jest modyfikowany. Wartosci 1 i -1 sa istotne
	      ze wzgledu na to, ze pierwiastek rownania kwadratowego jest
	      jednoczesnie cosinusem odpowiedniego kata.  */
	      if( ((*x1 <  1.0 + ROUND_OFF) && (*x1 >  1.0)) ) *x1 =  1.0;
	      if( ((*x1 > -1.0 - ROUND_OFF) && (*x1 < -1.0)) ) *x1 = -1.0;
	      return(LINEAR_SOLUTION);
   }
} /* Koniec QuadraticEquation() */



/***************************************************************************
*                                                                          *
*                             Ecos_Fsin_G                                  *
*                                                                          *
****************************************************************************

FUNKCJA:

Funkcja Ecos_Fsin_G rozwiazuje rownanie o postaci:
	 E cos(theta) + F sin(theta) - G = 0

Rozwiazanie:
  theta = atan2( g, +sqrt(e^2 + f^2 - g^2) ) - atan2(e,f)
lub
  theta = atan2( g, -sqrt(e^2 + f^2 - g^2) ) - atan2(e,f)

Ponadto sprawdza dopuszczalnosc otrzymanych wynikow.


WYWOlANIE:

   r = Ecos_Fsin_G(e, f, g, theta1_ptr, theta2_ptr,
		   lower_limit, upper_limit, max_theta_inc,
		   OldTheta, no_of_solutions);


DANE WEJSCIOWE:

 double e             - wspolczynnik E rownania: E cos(theta) + F sin(theta) - G = 0
 double f             - wspolczynnik F rownania: E cos(theta) + F sin(theta) - G = 0
 double g             - wspolczynnik G rownania: E cos(theta) + F sin(theta) - G = 0
 double lower_limit   - dolne ograniczenie narzucone na kat theta
 double upper_limit   - gorne ograniczenie narzucone na kat theta
 double max_theta_inc - maksymalny dopuszczalny przyrost ka`ta
			THETA w 1 kroku makro-interpolacji
 double OldTheta      - poprzednia wartosc wejsciowej
			wspolrzednej wewnetrznej

 int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				 byc obliczone wszystkie rozwiazania, czy
				 tez jedynie to lezace najblizej
				 poprzedniego



DANE WYJSCIOWE:

   double *theta1_ptr - wska`xniki na pierwsze rozwiazanie rownania
   double *theta2_ptr - wska`xniki na drugie rozwiazanie rownania

  Wartosc funkcji:
  r = NO_SOLUTION       - obliczone katy nie spelniaja ogranicze`n
						--> *theta1_ptr, *theta2_ptr
						    - nieokreslone
  r = ONE_SOLUTION      - e^2 + f^2 - g^2  = 0  --> *theta2_ptr
						    - nieokreslone
  r = TWO_SOLUTIONS     - e^2 + f^2 - g^2  > 0
  r = INCONSISTENT_DATA - e = 0, f = 0, g != 0  --> *theta1_ptr, *theta2_ptr
						    - nieokreslone
			- e^2 + f^2 - g^2  < 0  --> *theta1_ptr, *theta2_ptr
						    - nieokreslone
  r = TRIVIAL_EQUATION  - e = 0, f = 0, g  = 0  --> *theta2_ptr
						    - nieokreslone,
						    *theta1_ptr = OldTheta

---------------------------------------------------------------------------*/

/* o tyle moze byc przekroczone 0 przy sprawdzaniu rownania:
	 E cos(theta) + F sin(theta) - G = 0
   Spowodowane jest to kumulacja bledow numerycznych */
#define EQ_EPS 1.0e-4

/* o tyle moze byc przekroczone 1 lub -1 przez cos(THETA) ->
   spowodowane kumulacja bledow numerycznych */
#define COS_EPS 1.0e-15

/* o tyle moze byc przekroczone ograniczenie narzucone na THETA ->
   spowodowane kumulacja bledow numerycznych */
#define OFFSET 1.0e-4


int16_t model_5dof::Ecos_Fsin_G(double e, double f, double g,
			  double *theta1_ptr, double *theta2_ptr,
			  double lower_limit, double upper_limit,
			  double max_theta_inc, double OldTheta,
			  int16_t no_of_solutions)
{
  double r_2;  /* kwadrat promienia hipotetycznego okregu */
  double g_2;  /* g^2 */
  double efg;  /* e^2 + f^2 - g^2 */
  double fi;   /* hipotetyczny kat */
  double tf;   /* theta + fi */
  double pitf; /* PI - tf */
  double rt;   /* sqrt(r_2 - g_2) */

  int16_t wynik;  /* liczba rozwiaza`n rownania */

/* Poczatek Ecos_Fsin_G() */

 r_2 = e*e + f*f;
 g_2 = g*g;
 efg = r_2 - g_2;

/* sprawdzenie z jakim rownaniem mamy do czynienia */

 if( (r_2 < EQ_EPS) && (g_2 > EQ_EPS) )  /* e == 0, f == 0, g != 0 */
   {return(INCONSISTENT_DATA);}

 if( (r_2 < EQ_EPS) && (g_2 < EQ_EPS) )  /* e == 0, f == 0, g == 0 */
   { *theta1_ptr = OldTheta;  return(TRIVIAL_EQUATION); }


/* obliczenie hipotetycznego kata fi */
   if( (f < COS_EPS) && (f > -COS_EPS) ) /* cos(fi) == 0 */
       if(e > 0.0)                  /* sin(fi) > 0  ? */
	 fi = M_PI/2;                 /* sin(fi) > 0 */
       else  fi = -M_PI/2;            /* sin(fi) < 0 */
   else                          /* cos(fi) != 0 */
     fi = atan2(e,f);            /* cos(fi) != 0 */


/* dalsze sprawdzenie z jakim rownaniem mamy do czynienia */

 if( (efg > -EPS) && (efg < EPS) )  /* efg == 0 --> e^2 + f^2 - g^2  == 0 */
   {
     /* efg == 0 --> cos(theta+fi) == 0 */
     if(g > 0.0)                  /* sin(theta+fi) > 0  ? */
       *theta1_ptr = M_PI/2 - fi;                 /* sin(theta+fi) > 0 */
     else  *theta1_ptr = -M_PI/2 - fi;            /* sin(theta+fi) < 0 */
      /* normalizacja kata */
      if(*theta1_ptr > M_PI)
	*theta1_ptr -= 2.0*M_PI;
      if(*theta1_ptr < -M_PI + EQ_EPS)
	*theta1_ptr += 2.0*M_PI;
      if(*theta1_ptr > M_PI)
	*theta1_ptr = M_PI;
     /* sprawdzenie legalnosci kata theta */
      if ( (*theta1_ptr <= upper_limit) && (*theta1_ptr >= lower_limit) )
	if ( (no_of_solutions == ALL_SOLUTIONS) ||
	(fabs(*theta1_ptr - OldTheta)< max_theta_inc) )
     return(ONE_SOLUTION);
   else return(NO_SOLUTION);
   }
 else   /* efg != 0 --> e^2 + f^2 - g^2  ! 0 */
  if( efg < -EQ_EPS )  /* e^2 + f^2 - g^2  < 0 */
      return(INCONSISTENT_DATA);
  else  /* e^2 + f^2 - g^2  > 0 */
    {
      if( efg < 0 ) /* korekcja bledow numerycznych */
	efg = 0.0;
      rt = sqrt(efg);
      tf = atan2( g, rt );
      *theta1_ptr = tf - fi;
      /* normalizacja kata */
      if(*theta1_ptr > M_PI)
	*theta1_ptr -= 2.0*M_PI;
      if(*theta1_ptr < -M_PI + EQ_EPS)
	*theta1_ptr += 2.0*M_PI;
      if(*theta1_ptr > M_PI)
	*theta1_ptr = M_PI;
      if( tf > 0)
	 pitf = M_PI - tf;
      else  /* tf < 0  */
	 pitf = -M_PI - tf;
      *theta2_ptr = pitf - fi;
      /* normalizacja kata */
      if(*theta2_ptr > M_PI)
	*theta2_ptr -= 2.0*M_PI;
      if(*theta2_ptr < -M_PI + EQ_EPS)
	*theta2_ptr += 2.0*M_PI;
      if(*theta2_ptr > M_PI)
	*theta2_ptr = M_PI;

     /* sprawdzenie legalnosci kata theta1 i theta2 */

     /* wpierw kat theta2 */

      if ( (*theta2_ptr < upper_limit) && (*theta2_ptr > lower_limit) )
	/* w przestrzeni roboczej */
	if ( (no_of_solutions == ALL_SOLUTIONS) ||
	     (fabs(*theta2_ptr - OldTheta) < max_theta_inc) )
	  wynik = 2;
	else
	  wynik = 1;  /* przekroczenie dopuszczalnej predkosci lub pozadane pojedyncze rozwiazanie */
      else
	/* poza przestrzenia robocza */
	if ( (*theta2_ptr < upper_limit+OFFSET) && (*theta2_ptr > lower_limit-+OFFSET) )
	  { /* w niewielkim stopniu poza przestrzenia robocza => korekcja */
	    if(*theta2_ptr < upper_limit+OFFSET)
	      *theta2_ptr = lower_limit;
	    else
	      *theta2_ptr = upper_limit;
	    if ( (no_of_solutions == ALL_SOLUTIONS) ||
		 (fabs(*theta2_ptr - OldTheta) < max_theta_inc) ||
	     (fabs(fabs(*theta2_ptr - OldTheta) - 2.0*M_PI)< max_theta_inc) )
	      wynik = 2;
	    else
	      wynik = 1;  /* przekroczenie dopuszczalnej predkosci lub pozadane pojedyncze rozwiazanie */
	  }  /* koniec proby korekcji */
	else  /* korekcja jest niedopuszczalna */
	  wynik = 1;

     /* nastepnie kat theta1 */

      if ( (*theta1_ptr < upper_limit) && (*theta1_ptr > lower_limit) )
	/* w przestrzeni roboczej */
	if ( (no_of_solutions == ALL_SOLUTIONS) ||
	     (fabs(*theta1_ptr - OldTheta) < max_theta_inc) )
	   if(wynik == 2)
	     return(TWO_SOLUTIONS);  /* theta1 OK, theta2 tez bylo OK */
	   else
	      return(ONE_SOLUTION);  /* theta1 OK, theta2 nie bylo OK */
	else  /* przekroczenie dopuszczalnej predkosci lub pozadane pojedyncze rozwiazanie */
	  if(wynik == 2)
	    {
	      *theta1_ptr = *theta2_ptr;
	      return(ONE_SOLUTION);
	    }
	  else
	    return(NO_SOLUTION);  /* ani theta2 ani theta1 nie jest OK */
      else
	/* poza przestrzenia robocza */
	if ( (*theta1_ptr < upper_limit+OFFSET) && (*theta1_ptr > lower_limit-+OFFSET) )
	  { /* w niewielkim stopniu poza przestrzenia robocza => korekcja */
	    if(*theta1_ptr < upper_limit+OFFSET)
	      *theta1_ptr = lower_limit;
	    else
	      *theta1_ptr = upper_limit;
	    if ( (no_of_solutions == ALL_SOLUTIONS) ||
		 (fabs(*theta1_ptr - OldTheta) < max_theta_inc) )
	       if(wynik == 2)
		 return(TWO_SOLUTIONS);  /* theta1 OK, theta2 tez bylo OK */
	       else
		 return(ONE_SOLUTION);  /* theta1 OK, theta2 nie bylo OK */
	    else  /* przekroczenie dopuszczalnej predkosci lub pozadane pojedyncze rozwiazanie */
	      if(wynik == 2)
		{
		  *theta1_ptr = *theta2_ptr;
		  return(ONE_SOLUTION);  /* theta2 bylo OK, a theta1 nie jest OK */
		}
	      else
		return(NO_SOLUTION);  /* ani theta2 ani theta1 nie jest OK */
	  }  /* koniec proby korekcji */
	else  /* korekcja jest niedopuszczalna */
	  if(wynik == 2)
	    {
	      *theta1_ptr = *theta2_ptr;
	      return(ONE_SOLUTION);   /* theta2 bylo OK, a theta1 nie jest OK */
	    }
	  else
	    return(NO_SOLUTION);  /* ani theta2 ani theta1 nie jest OK */

    }  /* end else od if( efg < 0 ) */
    return (NO_SOLUTION);
} /* Koniec Ecos_Fsin_G() */



/***************************************************************************
*                                                                          *
*                         Check_cos_Theta1                                 *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Check_cos_Theta1 oblicza kat theta na podstawie wartosci
  funkcji cosinus tego kata oraz sprawdza, czy tak obliczony kat
  znajduje sie w zakre sie roboczym. Brane sa pod uwage obie
  mozliwosci: (cos,sin), (cos,-sin) - co najwyzej jedna jest
  wlasciwa. W przypadku poszukiwania tylko jednego rozwiazania
  sprawdza rowniez przekroczenie dopuszczalnej predkosci
  dla danego stopnia swobody.


WYWOlANIE:

  res = Check_cos_Theta1(cos_theta1, sin_theta1, theta1, p, r, t,
			 max_theta1_inc, no_of_solutions, OldTheta1);


DANE WEJSCIOWE:

  double cos_theta1   - cosinus kata theta

  double p            - wspolczynnik rownania weryfikujacego
  double r            - wspolczynnik rownania weryfikujacego
  double t            - wspolczynnik rownania weryfikujacego

  double max_theta1_inc - maksymalny dopuszczalny przyrost ka`ta
			 THETA1 w 1 kroku makro-interpolacji

  int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				  byc obliczone wszystkie rozwiazania, czy
				  tez jedynie to lezace najblizej
				  poprzedniego

  double OldTheta1      - poprzednia wartosc wejsciowej
			  wspolrzednej wewnetrznej


DANE WYJSCIOWE:

  double *theta1       - obliczony kat theta1
  double *sin_theta1   - sinus kata theta1

  Wartosci funkcji:
    int16_t res

    res = REJECT  - obliczony kat nie jest w przestrzeni roboczej
    res = ACCEPT  - obliczony kat jest w przestrzeni roboczej

---------------------------------------------------------------------------*/


// stara
// #define EQUATION_EPS 1.0E-5
// by Y - nowa
#define EQUATION_EPS 1.0E-10


int16_t model_5dof::Check_cos_Theta1(double cos_theta1, double *sin_theta1,
			       double *theta1,
			       double p, double r, double t,
			       double max_theta1_inc,
			       int16_t no_of_solutions,
			       double OldTheta1)

{
  double value;  /* wartosc lewej strony sprawdzanego rownania */

  /* Poczatek Check_cos_Theta1 */


  *sin_theta1 = sqrt(1.0 - cos_theta1*cos_theta1);

  value = p*cos_theta1*cos_theta1 + r*cos_theta1*(*sin_theta1) + t;
//	std::cout<<" VALUE: "<<value<<" sin_theta: "<< *sin_theta1 <<"\n";
  if((value > EQUATION_EPS) || (value < - EQUATION_EPS))
  {
      *sin_theta1 = -(*sin_theta1);
}

  if((cos_theta1 < EPS) && (cos_theta1> -EPS))  /* cos_theta1 = 0 */
     if(*sin_theta1 > 0.0)
       *theta1 = M_PI/2;
     else  *theta1 = -M_PI/2;
   else
     *theta1 = atan2(*sin_theta1,cos_theta1);


    /* czy *theta1 poza przestrzenia robocza? */

    if((*theta1 > UPPER_THETA1_LIMIT) || (*theta1 < LOWER_THETA1_LIMIT))
      return(REJECT);
     else
     {
      if(no_of_solutions == ALL_SOLUTIONS)
	 return(ACCEPT);
       else
	  if( fabs(*theta1 - OldTheta1) > max_theta1_inc )// w tym miejscu nie przechodzi, bo za duza roznica
	    return(REJECT);
	   else
	    return(ACCEPT);
	}  /* end: else od: if(no_of_solutions == ALL_SOLUTIONS) */
} /* Koniec Check_cos_Theta1 */



/***************************************************************************
*                                                                          *
*                              Check_Theta5                                *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Check_Theta5 sprawdza czy obliczony kat THETA5 spe~nia
  pierwotne r~wnania wektorowe, tzn.: A06*Q6 = Q0 oraz A06*V6 = V0,
  a ponadto gdy pozadane jest pojedyncze rozwiazanie sprawdza
  ograniczenia predkosciowe.


WYWOlANIE:

  res = Check_Theta5(theta5, c5, s5, q0, q6, v0, v6,
	      theta1_pointer, theta2_pointer, theta3_pointer, theta4_pointer,
	      no_of_solutions, max_theta5_inc, OldTheta5);


DANE WEJSCIOWE:

  double *theta5 - wska`xnik na THETA5
  double  c5     - cos(THETA5)
  double  s5     - sin(THETA5)

  double q0[3] - polozenie TCP w ukladzie bazowym
  double q6[3] - polozenie TCP w ukladzie kolnierza
  double v0[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem bazowego ukladu wspolrzednych
  double v6[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem ukladu wspolrzednych zwiazanego z kolnierzem

  THETA_NODE *theta1_pointer - wskaznik na strukture zawierajaca dane
			       dotyczace THETA1
  THETA_NODE *theta2_pointer - wskaznik na strukture zawierajaca dane
			       dotyczace THETA2
  THETA_NODE *theta3_pointer - wskaznik na strukture zawierajaca dane
			       dotyczace THETA3
  THETA_NODE *theta4_pointer - wskaznik na strukture zawierajaca dane
			       dotyczace THETA4
  int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				  byc obliczone wszystkie rozwiazania, czy
				  tez jedynie to lezace najblizej
				  poprzedniego
  double max_theta5_inc - maksymalny dopuszczalny przyrost ka`ta
			  THETA5 w 1 kroku makro-interpolacji
  double OldTheta5      - poprzednia wartosc wejsciowej
			  wspolrzednej wewnetrznej


DANE WYJSCIOWE:

  Wartosc funkcji:

   int16_t res

    res = ACCEPT
    res = REJECT

---------------------------------------------------------------------------*/
#undef  EQUATION_EPS
#define EQUATION_EPS 1.0E-2
#define ANGLE_EPS    1.0E-9

int16_t model_5dof::Check_Theta5(double *theta5, double c5, double s5,
		    double q0[3], double q6[3], double v0[3], double v6[3],
		    THETA_NODE *theta1_pointer, THETA_NODE *theta2_pointer,
		    THETA_NODE *theta3_pointer, THETA_NODE *theta4_pointer,
		    int16_t no_of_solutions,
		    double max_theta5_inc,
		    double OldTheta5)

{
  double nx;     /* pierwszy element pierwszego wiersza macierzy A06 */
  double ox;     /* drugi element pierwszego wiersza macierzy A06 */
  double ax;     /* trzeci element pierwszego wiersza macierzy A06 */
  double px;     /* czwarty element pierwszego wiersza macierzy A06 */
  double ny;     /* pierwszy element drugiego wiersza macierzy A06 */
  double oy;     /* drugi element drugiego wiersza macierzy A06 */
  double ay;     /* trzeci element drugiego wiersza macierzy A06 */
  double py;     /* czwarty element drugiego wiersza macierzy A06 */
  double nz;     /* pierwszy element trzeciego wiersza macierzy A06 */
  double oz;     /* drugi element trzeciego wiersza macierzy A06 */
  double az;     /* trzeci element trzeciego wiersza macierzy A06 */
  double s4s5;   /* sin(THETA4)*sin(THETA5) */
  double s4c5;   /* sin(THETA4)*cos(THETA5) */
  double ddd;    /* d6*os(THETA4) + a3*cos(THETA3) + a2*cos(THETA2) */
  double ee1;    /* A06*Q6 = Q0 ---> ee1 = Q0[X] ---> sluzy do weryfikacji rozwiazania rownania wyjsciowego */
  double ee2;    /* A06*Q6 = Q0 ---> ee2 = Q0[Y] ---> sluzy do weryfikacji rozwiazania rownania wyjsciowego  */
  double ee3;    /* A06*V6 = V0 ---> ee3 = V0[X] ---> sluzy do weryfikacji rozwiazania rownania wyjsciowego  */
  double ee4;    /* A06*V6 = V0 ---> ee4 = V0[Y] ---> sluzy do weryfikacji rozwiazania rownania wyjsciowego  */
  double ee5;    /* A06*V6 = V0 ---> ee5 = V0[Z] ---> sluzy do weryfikacji rozwiazania rownania wyjsciowego  */


/* Poczatek Check_Theta5() */

 /* Obliczenie poszczeg~lnych element~w macierzy A06 */
    ddd  = d6*C4 + a3*C3 + a2*C2;
    s4c5 = S4*c5;
    s4s5 = S4*s5;

    nx = C1*s4c5 + S1*s5;
    ny = S1*s4c5 - C1*s5;
    nz = C4*c5;

    ox = -C1*s4s5 + S1*c5;
    oy = -S1*s4s5 - C1*c5;
    oz = -C4*s5;

    ax = C1*C4;
    ay = S1*C4;
    az = -S4;

    px = C1*ddd;
    py = S1*ddd;

   /*  A06*Q6 = Q0   oraz A06*V6 = V0  */
    ee1 = nx*q6[X] + ox*q6[Y] + ax*q6[Z] + px - q0[X];
    ee2 = ny*q6[X] + oy*q6[Y] + ay*q6[Z] + py - q0[Y];
    ee3 = nx*v6[X] + ox*v6[Y] + ax*v6[Z] - v0[X];
    ee4 = ny*v6[X] + oy*v6[Y] + ay*v6[Z] - v0[Y];
    ee5 = nz*v6[X] + oz*v6[Y] + az*v6[Z] - v0[Z];

    if( ((ee1 < EQUATION_EPS) && (ee1 > -EQUATION_EPS)) &&
	((ee2 < EQUATION_EPS) && (ee2 > -EQUATION_EPS)) &&
	((ee3 < EQUATION_EPS) && (ee3 > -EQUATION_EPS)) &&
	((ee4 < EQUATION_EPS) && (ee4 > -EQUATION_EPS)) &&
	((ee5 < EQUATION_EPS) && (ee5 > -EQUATION_EPS)) )
      {
	if(no_of_solutions == ALL_SOLUTIONS)
	  return(ACCEPT); /* wszystkie rozwiazania */
	else
	  {
	    if (fabs(*theta5 - OldTheta5) < max_theta5_inc)
	      return(ACCEPT); /* jedno rozwiazanie i spelnione ograniczenia predkosciowe */
	    else
	      {
		if( (*theta5 < M_PI + ANGLE_EPS) &&  (*theta5 > M_PI - ANGLE_EPS) )
		  /* THETA5 = +PI, mozna jeszcze sprobowac z -PI */
		  *theta5 = -M_PI;
		else
		  if( (*theta5 < -M_PI + ANGLE_EPS) &&  (*theta5 > -M_PI - ANGLE_EPS) )
		    /* THETA5 = -PI, mozna jeszcze sprobowac z +PI */
		    *theta5 = M_PI;
		  else
		  {
		  // std::cout << "1" << std::endl;
		    return(REJECT);
		    }
		if (fabs(*theta5 - OldTheta5) < max_theta5_inc)
		  return(ACCEPT); /* jedno rozwiazanie i spelnione ograniczenia predkosciowe */
		else
		{
		// std::cout << "2" << std::endl;
		  return(REJECT);
		  }
	      }
	  } /* koniec else od: if(no_of_solutions == ALL_SOLUTIONS) */
      } /* koniec od: if( ((ee1 < EQUATION_EPS) && ... ))) */
    else
    {
    // std::cout << "3" << std::endl;
       return(REJECT);
       }

} /* Koniec Check_Theta5() */



/***************************************************************************
*                                                                          *
*                              Flange                                      *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Flange oblicza polozenie srodka kolnierza w trakcie
  rozwiazywania odwrotnego zagadnienia kinematycznego dla robota IRp-6.


WYWOlANIE:

   r = Flange(u0, v0, u6, radius_2, theta1_pointer, theta4_pointer,
	       o06_prim, o06_bis);


DANE WEJSCIOWE:

  THETA_NODE *theta1_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA1, dla ktorego obliczane jest
			       polozenie srodka kolnierza
  THETA_NODE *theta4_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA4, dla ktorego obliczane jest
			       polozenie srodka kolnierza

  double radius_2 - kwadrat dlugosci promienia okregu

  double v0[3] - wersor (swobodny) orientacji osi narzedzia
		 wzgledem bazowego ukladu odniesienia
  double u0[3] - wektor (zwiazany) srodka okregu, na ktorym moze
		 znajdowac sie kolnierz, wzgledem bazowego ukladu
		 odniesienia
  double u6[3] - wektor (zwiazany) srodka okregu, na ktorym moze
		 znajdowac sie kolnierz, wzgledem ukladu wspolrzednych
		 zwiazanego z kolnierzem


DANE WYJSCIOWE:

  Wartosci funkcji:
    int16_t r

    r = NO_SOLUTION
    r = ONE_SOLUTION
    r = TWO_SOLUTIONS

  double o06_prim[3] - wspolrzedne srodka kolnierza (tzn. poczatku
		       ukladu 6) wzgledem ukladu bazowego (tzn. ukladu 0)
		       - pierwsze rozwiazanie.
  double o06_bis[3]  - wspolrzedne srodka kolnierza (tzn. poczatku
		       ukladu 6) wzgledem ukladu bazowego (tzn. ukladu 0)
		       - drugie rozwiazanie (jezeli istnieje).

---------------------------------------------------------------------------*/

#define A 0
#if defined(B)
  #undef B
#endif
#define B 1
#define D 2

#undef  EPS
#define EPS 1.0E-8
#define EEPS 1.0E-10

int16_t model_5dof::Flange(double u0[3], double v0[3], double u6[3],
		     double radius_2,
		     THETA_NODE *theta1_pointer, THETA_NODE *theta4_pointer,
		     double o06_prim[3], double o06_bis[3])
{

  double abd_1[3]; /* dane pomocne przy wyznaczaniu wspolrzednych srodka
		      kolnierza (tzn. poczatku ukladu 6) wzgledem ukladu
		      bazowego (tzn. ukladu 0) - pierwsze rozwiazanie. */
  double abd_2[3]; /* dane pomocne przy wyznaczaniu wspolrzednych srodka
		      kolnierza (tzn. poczatku ukladu 6)  wzgledem ukladu
		      bazowego (tzn. ukladu 0) - drugie rozwiazanie (jezeli
		      istnieje). */

   double kkk;     /* zmienna pomocnicza */
   double sss;     /* zmienna pomocnicza */
   double xxx;     /* zmienna pomocnicza */
   double aaa;     /* zmienna pomocnicza */
   double ww;      /* wartosc wyznacznika glownego, gdy v0[Z]=0 */
   double w;       /* wartosc wyznacznika glownego */
   int16_t result;      /* do przechowywania wyniku dzialania funkcji
				 Flange_exception */

/* Poczatek Flange() */

   ww = v0[X]*C1 + v0[Y]*S1;
   w  = v0[Z]*C4 + ww*S4;

   if( (v0[Z] > EPS) || (v0[Z] < -EPS) )   /* v0[Z] != 0 */
     {  /* v0[Z] != 0 */
      if( (w > EPS) || (w < -EPS) )   /* w != 0 */
	{  /* w != 0 */
	  sss = u0[X]*S1 - u0[Y]*C1;
	  xxx = -u6[Z]*v0[Z];
	  aaa = v0[Z]*C4;
	  abd_1[A] = (xxx*C1 - sss*(aaa*S1 + v0[Y]*S4))/(-aaa - ww*S4);
	  abd_1[B] = (xxx*S1 + sss*(aaa*C1 + v0[X]*S4))/(-aaa - ww*S4);
	  abd_1[D] = (abd_1[A]*v0[X] + abd_1[B]*v0[Y])/(-v0[Z]);
	  result = ONE_SOLUTION;
	}
       else
	{  /* w == 0 */
	 result = Flange_exception(u0, v0, u6, radius_2,
	   theta1_pointer, theta4_pointer, abd_1, abd_2);
  }  /* end if(w...) */
     }  /* end: v0[Z] != 0 */
    else
     {  /* v0[Z] == 0 */
      if( (ww > EPS) || (w < -EPS) )   /* ww != 0 */
	{  /* ww != 0 */
	  kkk = (u0[X]*S1 - u0[Y]*C1)/ww;
	  abd_1[A] = kkk*v0[Y];
	  abd_1[B] = -kkk*v0[X];
	  if( (S4 > EEPS) || (S4 < -EEPS) )   /* S4 != 0 */
	   {   /* S4 != 0 */
	     abd_1[D] = (u6[Z] - C4*kkk*(v0[X]*S1 - v0[Y]*C1))/S4;
	     result = ONE_SOLUTION;
	   }
	  else
	   {   /* S4 == 0 */
	    abd_2[A] = abd_1[A];
      abd_2[B] = abd_1[B];
      if((radius_2 + EEPS) < (abd_1[A]*abd_1[A]+abd_1[B]*abd_1[B]))
      return(NO_SOLUTION);
      aaa = radius_2 - abd_1[A]*abd_1[A] - abd_1[B]*abd_1[B];
      if(aaa > 0.0)
	 abd_1[D] = sqrt(aaa);
      else abd_1[D] = 0.0;
      abd_2[D] = - abd_1[D];
      result = TWO_SOLUTIONS;
     }
  }  /* end: ww != 0 */
       else
	{  /* ww == 0 */
	 result = Flange_exception(u0, v0, u6, radius_2,
	  theta1_pointer, theta4_pointer, abd_1, abd_2);
  }  /* end if(ww...) */

     }  /* end: v0[Z] == 0 */

/* obliczenie wspolrzednych srodka kolnierza */
  switch(result)
   {
    case NO_SOLUTION: return(NO_SOLUTION);
    case ONE_SOLUTION:
	 o06_prim[X] = u0[X] - abd_1[A];
	 o06_prim[Y] = u0[Y] - abd_1[B];
	 o06_prim[Z] = u0[Z] - abd_1[D];
	 return(ONE_SOLUTION);
    case TWO_SOLUTIONS:
	 o06_prim[X] = u0[X] - abd_1[A];
	 o06_prim[Y] = u0[Y] - abd_1[B];
	 o06_prim[Z] = u0[Z] - abd_1[D];
	 o06_bis[X] = u0[X] - abd_2[A];
	 o06_bis[Y] = u0[Y] - abd_2[B];
	 o06_bis[Z] = u0[Z] - abd_2[D];
	 return(TWO_SOLUTIONS);
    default: return(NO_SOLUTION);
   }  /* end switch(result)  */

} /* Koniec Flange() */



/***************************************************************************
*                                                                          *
*                         Flange_exception                                 *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Flange_exception dokonuje pomocniczych przeksztalce`n przy
  obliczaniu polozenia srodka kolnierza (w przypadku szczegolnym) w
  trakcie rozwiazywania odwrotnego zagadnienia kinematycznego dla
  robota IRp-6.


WYWOlANIE:

   r = Flange_exception(u0, v0, u6, radius_2, theta1_pointer, theta4_pointer,
			abd_1, abd_2);


DANE WEJSCIOWE:

  THETA_NODE *theta1_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA1, dla ktorego obliczane jest
			       polozenie srodka kolnierza
  THETA_NODE *theta4_pointer - wskaznik na zestaw danych dotyczacych
			       kata THETA4, dla ktorego obliczane jest
			       polozenie srodka kolnierza

  double radius_2 - kwadrat dlugosci promienia okregu

  double v0[3]    - wersor (swobodny) orientacji osi narzedzia
		    wzgledem bazowego ukladu odniesienia
  double u0[3]    - wektor (zwiazany) srodka okregu, na ktorym moze
		    znajdowac sie kolnierz, wzgledem bazowego ukladu
		    odniesienia
  double u6[3]    - wektor (zwiazany) srodka okregu, na ktorym moze
		    znajdowac sie kolnierz, wzgledem ukladu
		    wspolrzednych zwiazanego z kolnierzem


DANE WYJSCIOWE:

  Wartosci funkcji:
    int16_t r

    r = NO_SOLUTION
	 r = TRIVIAL_EQUATION
    r = ONE_SOLUTION
    r = TWO_SOLUTIONS

  double abd_1[3] - dane pomocne przy wyznaczaniu wspolrzednych srodka
		    kolnierza (tzn. poczatku ukladu 6) wzgledem ukladu
		    bazowego (tzn. ukladu 0) - pierwsze rozwiazanie.
  double abd_2[3] - dane pomocne przy wyznaczaniu wspolrzednych srodka
		    kolnierza (tzn. poczatku ukladu 6)  wzgledem ukladu
		    bazowego (tzn. ukladu 0) - drugie rozwiazanie (jezeli
		    istnieje).

---------------------------------------------------------------------------*/

#define A 0
#define B 1
#define D 2

#undef  EPS
#define EPS 1.0E-8
#undef  EEPS
#define EEPS 1.0E-7


int16_t model_5dof::Flange_exception(double u0[3], double v0[3], double u6[3],
	     double radius_2,
	 THETA_NODE *theta1_pointer, THETA_NODE *theta4_pointer,
	   double abd_1[3], double abd_2[3])
{
   int16_t result;  /* do przechowywania wyniku dzialania funkcji
			     rozwiazujacej rownanie kwadratowe */
   double delta;                /* wyroznik rownania kwadratowego */
   double uuu;            /* u0[Y]*cos(THETA1) - u0[X]*sin(THETA1) */
   double uuu_2;          /* kwadrat uuu */
   double u6z_2;          /* kwadrat u6[Z] */
   double c4_2;           /* cos(THETA4)^2 */
   double d_2;            /* zmienna pomocnicza */
   double c1c4;           /* cos(THETA1)*cos(THETA4) */
   double c1s4;           /* cos(THETA1)*sin(THETA4) */
   double s1s4;           /* sin(THETA1)*sin(THETA4) */
   double s1c4;           /* sin(THETA1)*cos(THETA4) */
   double c1c4uuu;        /* cos(THETA1)*cos(THETA4)*uuu */
   double s1c4uuu;        /* sin(THETA1)*cos(THETA4)*uuu */
   double u6zs1;          /* u6[Z]*sin(THETA1) */
   double u6zc1;          /* u6[Z]*cos(THETA1) */
   double radius;         /* promie`n hipotetycznego okregu */

/* Poczatek Flange_exception() */

  uuu = u0[Y]*C1 - u0[X]*S1;
  uuu_2 = uuu*uuu;
  if( (C4 > EEPS) || (C4 < -EEPS) )   /* C4 != 0 */
    {   /* C4 != 0 */

       u6z_2 = u6[Z]*u6[Z];
       c4_2 = C4*C4;
       delta = -4.0 * c4_2 * ( u6z_2 + uuu_2 - radius_2);

       result = QuadraticEquation(1.0, 2*u6[Z]*S4, u6z_2 + c4_2*(uuu_2 - radius_2),
				  &abd_1[D], &abd_2[D],delta);
       switch(result)
	 {
	   case NO_SOLUTION:
	   case INCONSISTENT_DATA:
		return(NO_SOLUTION);
	   case TRIVIAL_EQUATION:
		return(TRIVIAL_EQUATION);
	   case ONE_SOLUTION:
	   case LINEAR_SOLUTION:
	   abd_1[A] = (abd_1[D]*C1*C4 + u6[Z]*C1 - S1*C4*uuu)/C4;
	   abd_1[B] = (abd_1[D]*C1*S4 + u6[Z]*S1 + C1*C4*uuu)/C4;
		return(ONE_SOLUTION);
	   case TWO_SOLUTIONS:

		s1s4 = S1*S4;
		c1s4 = C1*S4;
		c1c4 = C1*C4;
		s1c4 = S1*C4;
		s1c4uuu = S1*C4*uuu;
		c1c4uuu = c1c4*uuu;
		u6zc1 = u6[Z]*C1;
		u6zs1 = u6[Z]*S1;

	     /* pierwsze rozwiazanie */
	   abd_1[A] = (abd_1[D]*c1c4 + u6zc1 - s1c4uuu)/C4;
	   abd_1[B] = (abd_1[D]*s1s4 + u6zs1 + c1c4uuu)/C4;
	     /* drugie rozwiazanie */
	   abd_2[A] = (abd_2[D]*c1s4 + u6zc1 - s1c4uuu)/C4;
	   abd_2[B] = (abd_2[D]*s1s4 + u6zs1 + c1c4uuu)/C4;
		return(TWO_SOLUTIONS);
	   default :
		return(NO_SOLUTION);
	  }  /* end switch(result) */

    }
   else
    {   /* C4 == 0 */

       abd_1[D] = -u6[Z]/S4;
       abd_2[D] = abd_1[D];
       d_2 = abd_1[D]*abd_1[D];
       if ((radius_2 - d_2 - uuu_2) > 0.0)
	    radius = sqrt(radius_2 - d_2 - uuu_2);
       else
	if ((radius_2 - d_2 - uuu_2) > -EPS) radius = 0.0;
	else return(NO_SOLUTION);
	abd_1[B] = C1*uuu - radius*S1;
	abd_2[B] = C1*uuu + radius*S1;
	abd_1[A] = -S1*uuu - radius*C1;
	abd_2[A] = -S1*uuu + radius*C1;
		return(TWO_SOLUTIONS);

    }  /* end if(...C4...) */


} /* Koniec Flange_exception() */



/***************************************************************************
*                                                                          *
*                              Add_Theta                                   *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Add_Theta tworzy kolejny wezel drzewa rozwiaza`n
  odwrotnego zagadnienia kinematycznego. Wypelnia go danymi.


WYWOlANIE:

  r = Add_Theta(Theta, cosTheta, sinTheta);


DANE WEJSCIOWE:

  double Theta      - wartosc kata theta
  double cosTheta   - cosinus kata theta
  double sinTheta   - sinus kata theta


DANE WYJSCIOWE:

  Wartosci funkcji:
  THETA_NODE *Add_Theta() - tworzony wezel

    r = wskaznnik    - wezel zostal stworzony
    r = NULL          - zabraklo pamieci na stworzenie wezla

---------------------------------------------------------------------------*/

THETA_NODE* model_5dof::Add_Theta(double Theta, double cosTheta, double sinTheta)

{
   THETA_NODE *Theta_node;

  /* Poczatek Add_Theta */

  /* stworzenie wezla */
  if( (Theta_node = (THETA_NODE *)malloc(sizeof(THETA_NODE))) == NULL)
    return(NULL);

  /* wypelnienie wezla danymi */
  Theta_node->value = Theta;           /* wartosc kata theta */
  Theta_node->cos_theta = cosTheta;    /* cosinus  kata theta */
  Theta_node->sin_theta = sinTheta;    /* sinus  kata theta */
  Theta_node->NextTheta = NULL;        /* wskaznik na nastepna
					  wartosc tego samego kata */
  Theta_node->NextAngle = NULL;        /* wskaznik na nastepny kat */

  return(Theta_node);

} /* Koniec Add_Theta */



/***************************************************************************
*                                                                          *
*                              Create1                                     *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Create1 tworzy fragment drzewa rozwiaza`n dla jednego z
  pierwiastkow rownania kwadratowego dotyczacego kata THETA1.

  UWAGA! Funkcja tworzy co najwyzej dwa wezly i zwraca wskaznik
	  do korzenia tak powstalego poddrzewa.

WYWOlANIE:

  ptr = Create1(x, p, r, t, result, max_theta_inc, no_of_solutions, OldTheta);


DANE WEJSCIOWE:

  double x  - pierwiastek rownania kwadratowego
  double p  - wspolczynnik rownania:  p cos^2 + r cos sin + t = 0
  double r  - wspolczynnik rownania:  p cos^2 + r cos sin + t = 0
  double t  - wspolczynnik rownania:  p cos^2 + r cos sin + t = 0

  double max_theta_inc - maksymalny dopuszczalny przyrost ka`ta
			 THETA w 1 kroku makro-interpolacji

  int16_t no_of_solutions - zmienna booleowska okreslajaca, czy maja
				  byc obliczone wszystkie rozwiazania, czy
				  tez jedynie to lezace najblizej
				  poprzedniego

  double OldTheta - poprzednia wartosc wejsciowej wspolrzednej
		    wewnetrznej


DANE WYJSCIOWE:

  Wartosc funkcji:
   THETA_NODE *ptr   - wskaznik na korze`n w poddrzewie rozwiaza`n

   int16_t *result
    *result = OUT_OF_RANGE
    *result = ONE_NODE
    *result = TWO_NODES
    *result = INVALID_ROOT_X
    *result = OUT_OF_MEMORY

---------------------------------------------------------------------------*/

/* o tyle moze byc przekroczone ograniczenie narzucone na THETA ->
   spowodowane kumulacja bledow numerycznych */
#define OFFSET 1.0e-4

THETA_NODE* model_5dof::Create1(double x, double p, double r, double t,
		    int16_t *result,
		    double max_theta_inc, int16_t no_of_solutions,
		    double OldTheta)
{
   double s;                  /* sin(THETA1) */
   double c;                  /* cos(THETA1) */
   double theta;              /* THETA1 */
   THETA_NODE *root_pointer;  /* wskaznik na korze`n w poddrzewie
				 rozwiaza`n */
   THETA_NODE *node_pointer;  /* wskaznik na wezel w drzewie
				 rozwiaza`n  */



  /* Poczatek Create1() */
  /* inicjalizacja wskaznikow */
     root_pointer = NULL;
     node_pointer = NULL;

 	if((x > (1.0 + EPS)) || (x < 0.0)){
		*result = INVALID_ROOT_X;
		return(NULL);
	} /* bo x = (cos THETA1)^2 */

  	if( x > 1.0)
		x = 1.0;
  c = sqrt(x);

  if( Check_cos_Theta1(c, &s, &theta, p, r, t, max_theta_inc, no_of_solutions, OldTheta) == ACCEPT ) {/* ACCEPT Check_cos_Theta1 */

	      if((root_pointer = Add_Theta(theta,c,s)) == NULL)
		 	{*result = OUT_OF_MEMORY; return(NULL); }

	      /* sprawdzenie drugiego rozwiazania */
	      if(theta < 0.0)
				theta += M_PI;
	      else theta -= M_PI;
	     if((theta > UPPER_THETA1_LIMIT) ||	 (theta < LOWER_THETA1_LIMIT))
		{

			if( (theta > UPPER_THETA1_LIMIT+OFFSET) || (theta < LOWER_THETA1_LIMIT-OFFSET) )
			    { *result = ONE_NODE;

			       return(root_pointer);
			    }
			else
			    if(theta < UPPER_THETA1_LIMIT)
			      	 theta = LOWER_THETA1_LIMIT;
			    else
			     	  theta = UPPER_THETA1_LIMIT;
		}
	      if( (no_of_solutions == SINGLE_SOLUTION) && (fabs(theta - OldTheta) > max_theta_inc) )	{

			*result = ONE_NODE;
			return(root_pointer);
		 }
	     if((node_pointer = Add_Theta(theta,-c,-s)) == NULL){

			*result = OUT_OF_MEMORY;
			return(NULL);
		}
	      root_pointer->NextTheta = node_pointer;
	      *result = TWO_NODES;

	      return(root_pointer);

    } /* end ACCEPT Check_cos_Theta1 */

   else{ /* REJECT Check_cos_Theta1 */

      if(theta < 0.0)
		theta += M_PI;
      else
		 theta -= M_PI;

      if((theta > UPPER_THETA1_LIMIT) ||	 (theta < LOWER_THETA1_LIMIT)){
		  if( (theta > UPPER_THETA1_LIMIT+OFFSET) || (theta < LOWER_THETA1_LIMIT-OFFSET) ){
		       *result = OUT_OF_RANGE;
		       return(NULL);
		   }
	  	else
		    if(theta < UPPER_THETA1_LIMIT)
		       theta = LOWER_THETA1_LIMIT;
		    else
		       theta = UPPER_THETA1_LIMIT;
	 }

      if( (no_of_solutions == SINGLE_SOLUTION) && (fabs(theta - OldTheta) > max_theta_inc) ){
		 *result = OUT_OF_RANGE;
		return(NULL);
	 }
      if((root_pointer = Add_Theta(theta,-c,-s)) == NULL){
		*result = OUT_OF_MEMORY;
		return(NULL);

 	 }

      *result = ONE_NODE;
       return(root_pointer);
    } /* end REJECT Check_cos_Theta1 */
} /* Koniec Create1() */



/***************************************************************************
*                                                                          *
*                       Delete_Theta_Tree                                  *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Delete_Theta_Tree kasuje dane zawarte w drzewie rozwiaza`n
  odwrotnego zagadnienia kinematycznego jednoczesnie zwracajac
  zajeta pamiec. Wykorzystuje algorytm rekursywny.


WYWOlANIE:

  Delete_Theta_Tree(root_ptr);


DANE WEJSCIOWE:

  THETA_NODE *root_ptr   - wskaznik na korze`n poddrzewa


DANE WYJSCIOWE:   brak

---------------------------------------------------------------------------*/

void model_5dof::Delete_Theta_Tree(THETA_NODE *root_ptr)

{
    THETA_NODE *node_ptr;   /* wskaznik biezacego wezla warstwy */
    THETA_NODE *next_node;  /* wskaznik kolejnego wezla warstwy */


  /* Poczatek Delete_Theta_Tree() */

  node_ptr = root_ptr;

  while(node_ptr != NULL)
  {
    if(node_ptr->NextAngle != NULL)
      {
	/* skasuj poddrzewo */
	Delete_Theta_Tree(node_ptr->NextAngle);
      }

    /* skasuj wezel biezacy */
    next_node = node_ptr->NextTheta;
    free(node_ptr);
    node_ptr = next_node;

  }  /* end while */

} /* Koniec Delete_Theta_Tree() */



/***************************************************************************
*                                                                          *
*                      Extract_vect_from_tree                              *
*                                                                          *
****************************************************************************

FUNKCJA:

  Funkcja Extract_vect_from_tree przepisuje pierwsze rozwiazanie
  (wspolrzedne wewnetrzne - katy THETA - oraz ich sinusy i cosinusy)
  zawarte w drzewie rozwiaza`n odwrotnego zagadnienia kinematycznego do
  adekwatnych wektorow. Zaklada sie, ze drzewo jest poprawnie zbudowane.


WYWOlANIE:

  Extract_vect_from_tree(root_ptr, Theta, tsin, tcos);


DANE WEJSCIOWE:

  THETA_NODE *root_ptr   - wskaznik na korze`n drzewa


DANE WYJSCIOWE:

  double Theta[5] - wartosci katow theta (katy umieszczone sa w
		    tablicy w porzadku: THETA1, THETA2, THETA3,
		    THETA4, THETA5.
  double tsin[5] - wartosci funkcji sinus wspolrzednych, w kolejnosci
		   THETA1, THETA2, THETA23, THETA234, THETA5.
  double tcos[5] - wartosci funkcji cosinus wspolrzednych, w kolejnosci
		   THETA1, THETA2, THETA23, THETA234, THETA5.

---------------------------------------------------------------------------*/

void model_5dof::Extract_vect_from_tree(THETA_NODE *root_ptr, double Theta[5],
			    double tsin[5], double tcos[5])

{
    THETA_NODE *node_ptr;   /* wskaznik biezacego wezla */

 /* Poczatek Extract_vect_from_tree() */

  /* THETA_1 */
  node_ptr = root_ptr;
  Theta[0] = node_ptr->value;
  tsin[0]  = node_ptr->sin_theta;
  tcos[0]  = node_ptr->cos_theta;
  /* THETA_4 */
  node_ptr = node_ptr->NextAngle;
  Theta[3] = node_ptr->value;
  tsin[3]  = node_ptr->sin_theta;
  tcos[3]  = node_ptr->cos_theta;

  /* THETA_2 */
  node_ptr = node_ptr->NextAngle;
  Theta[1] = node_ptr->value;
  tsin[1]  = node_ptr->sin_theta;
  tcos[1]  = node_ptr->cos_theta;

  /* THETA_3 */
  node_ptr = node_ptr->NextAngle;
  Theta[2] = node_ptr->value;
  tsin[2]  = node_ptr->sin_theta;
  tcos[2]  = node_ptr->cos_theta;

  /* THETA_5 */
  node_ptr = node_ptr->NextAngle;
  Theta[4] = node_ptr->value;
  tsin[4]  = node_ptr->sin_theta;
  tcos[4]  = node_ptr->cos_theta;

  return;

} /* Koniec Extract_vect_from_tree() */

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp
