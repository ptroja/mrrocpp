// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6m_5dof.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na torze
//				- deklaracja klasy
//				- wykorzystanie toru jako czynnego stopnia swobody
//
// Autor:		tkornuta
// Data:		31.01.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6M_KIN_MODEL_5DOF)
#define _IRP6M_KIN_MODEL_5DOF

// Definicja klasy kinematic_model.
#include "robot/irp6m/kinematic_model_irp6m_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6m {

// Zakresy ruchu poszczegolnych stopni swobody (w radianach lub milimetrach)
#define UPPER_THETA1_LIMIT  170.0*M_PI/180.0 // [rad]
#define LOWER_THETA1_LIMIT -170.0*M_PI/180.0

#define UPPER_THETA2_LIMIT -50.0*M_PI/180.0
#define LOWER_THETA2_LIMIT -130.0*M_PI/180.0

#define UPPER_THETA3_LIMIT  40.0*M_PI/180.0
#define LOWER_THETA3_LIMIT -25.0*M_PI/180.0

#define UPPER_THETA4_LIMIT  90.1*M_PI/180.0
#define LOWER_THETA4_LIMIT -90.1*M_PI/180.0

/* ruch piatego stopnia swobody nie jest ograniczony */
#define UPPER_THETA5_LIMIT  10.0 //M_PI
#define LOWER_THETA5_LIMIT -10.0 // -M_PI



#define S1 theta1_pointer->sin_theta
#define C1 theta1_pointer->cos_theta
#define S2 theta2_pointer->sin_theta
#define C2 theta2_pointer->cos_theta
#define S3 theta3_pointer->sin_theta
#define C3 theta3_pointer->cos_theta
#define S4 theta4_pointer->sin_theta
#define C4 theta4_pointer->cos_theta
#define S5 theta5_pointer->sin_theta
#define C5 theta5_pointer->cos_theta

#define INTER_PERIOD 10000

/* ***********************************************************************
      Wartosci zwracane przez funkcje uczestniczace w rozwiazywaniu
     prostego i odwrotnego zagadnienia kinematycznego.
 ************************************************************************* */
/* Rozwiazanie rownania kwadratowego
            lub rownania --> E cos + F sin - G = 0 */

#define NO_SOLUTION        0
#define ONE_SOLUTION       1
#define TWO_SOLUTIONS      2
#define LINEAR_SOLUTION    3
#define INCONSISTENT_DATA  4
#define TRIVIAL_EQUATION   5

/* wartosci zwracane przez funkcje Create1 */
#define ONE_NODE         21
#define TWO_NODES        22
#define OUT_OF_RANGE     23
#define INVALID_ROOT_X   24

/* Sprawdzenie pary (cos,sin) dla pewnego kata theta */
#define ACCEPT  1
#define REJECT  0

/* pozadana liczba rozwiaza`n odwrotnego zagadnienia kinematycznego */
#define SINGLE_SOLUTION 0
#define ALL_SOLUTIONS   1


#if defined(X)
  #undef X
#endif
#if defined(Y)
  #undef Y
#endif
#define X 0
#define Y 1
#define Z 2
#define EPS 1.0E-15

/* maksymalne predkosci ruchu poszczegolnych stopni swobody w [rad/s] */

#define MAX_THETA1_VELOCITY 1.6
#define MAX_THETA2_VELOCITY 1.6
#define MAX_THETA3_VELOCITY 1.6
#define MAX_THETA4_VELOCITY 1.6
#define MAX_THETA5_VELOCITY 1.6

/* -------------------------------------------------------------------------
  Kody bledow wykrywanych przez procedury realizujace przeliczniki
  dla robota IRp-6 o pieciu stopniach swobody
 -------------------------------------------------------------------------- */

          /* BleDY NIE FATALNE */

  /* wynik dzialania funkcji Add_Theta */
#define OUT_OF_MEMORY                   -1

  /* wartosci zwracane przez funkcje Theta_i, i = 1..5 */
#define NEGATIVE_DISCRIMINANT_THETA1    -2
#define INCONSISTENT_DATA_THETA1        -3
#define UNKNOWN_ERROR_THETA1            -4
#define INVALID_ROOT_THETA1             -5
#define INVALID_ROOTS_THETA1            -6
#define INCONSISTENT_DATA_THETA2        -7
#define UNKNOWN_ERROR_THETA2            -8
#define THETA2_OUT_OF_RANGE             -9
#define ATAN_THETA3_ERROR               -10
#define THETA3_OUT_OF_RANGE             -11
#define UNKNOWN_ERROR_THETA4            -12
#define INCONSISTENT_DATA_THETA4        -13
#define THETA4_OUT_OF_RANGE             -14
#define INVALID_FLANGE                  -15
#define UNKNOWN_ERROR_THETA5            -16
#define THETA5_OUT_OF_RANGE             -17
#define INCONSISTENT_DATA_THETA5        -18
#define THETA5_EQUATION_VIOLATION       -19
#define WRONG_PERIOD_CODE               -21

/* Wezel drzewa zawierajacego wszystkie dopuszczalne rozwiazania
   odwrotnego zagadnienia kinematycznego */
typedef struct AngleTheta
{
  double            value;        /* wartosc kata theta */
  double            cos_theta;    /* cosinus  kata theta */
  double            sin_theta;    /* sinus  kata theta */
  struct AngleTheta *NextTheta;   /* wskaznik na nastepna
                                     wartosc tego samego kata */
  struct AngleTheta *NextAngle;   /* wskaznik na nastepny kat */
} THETA_NODE;


class model_5dof : public model_with_wrist
{
protected:
  // Wysokosc kolumny.
  double d1;

  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void);




THETA_NODE *Theta_1(double q0[3], double v0[3],
		    double q6[3], double v6[3],
		    int16_t *result,
		    double old_theta[5],
		    double interpolation_period,
		    int16_t no_of_solutions);

THETA_NODE *Theta_2(THETA_NODE *theta1_pointer, THETA_NODE *theta4_pointer,
	    double u0[3], double v0[3], double q0[3], double u6[3],
	    double q6[3], double v6[3], double radius_2,
	    int16_t *result, double old_theta[5],
	    double interpolation_period, int16_t no_of_solutions);

THETA_NODE* Theta_3(THETA_NODE *theta1_pointer,
		    THETA_NODE *theta2_pointer,
		    THETA_NODE *theta4_pointer,
		    double e, double f,
		    double q0[3], double q6[3],
		    double v0[3], double v6[3],
		    int16_t *result, double old_theta[5],
		    double interpolation_period,
		    int16_t no_of_solutions);

THETA_NODE* Theta_4(THETA_NODE *theta1_pointer,
		    double u0[3], double v0[3], double u6[3], double v6[3],
		    double q0[3], double q6[3], double radius_2,
		    int16_t *result, double old_theta[5],
		    double interpolation_period,
		    int16_t no_of_solutions);

THETA_NODE* Theta_5(THETA_NODE *theta1_pointer,
		    THETA_NODE *theta2_pointer,
		    THETA_NODE *theta3_pointer,
		    THETA_NODE *theta4_pointer,
		    double q0[3], double q6[3],
		    double v0[3], double v6[3],
		    int16_t *result, double old_theta[5],
		    double interpolation_period,
		    int16_t no_of_solutions);

int16_t QuadraticEquation(double a, double b, double c,
				double *x1, double *x2, double delta);

int16_t Ecos_Fsin_G(double e, double f, double g,
			  double *theta1_ptr, double *theta2_ptr,
			  double lower_limit, double upper_limit,
			  double max_theta_inc, double OldTheta,
			  int16_t no_of_solutions);

int16_t Check_cos_Theta1(double cos_theta1, double *sin_theta1,
			       double *theta1,
			       double p, double r, double t,
			       double max_theta1_inc,
			       int16_t no_of_solutions,
			       double OldTheta1);

int16_t Check_Theta5(double *theta5, double c5, double s5,
		    double q0[3], double q6[3], double v0[3], double v6[3],
		    THETA_NODE *theta1_pointer, THETA_NODE *theta2_pointer,
		    THETA_NODE *theta3_pointer, THETA_NODE *theta4_pointer,
		    int16_t no_of_solutions,
		    double max_theta5_inc,
		    double OldTheta5);

int16_t Flange(double u0[3], double v0[3], double u6[3],
		     double radius_2,
		     THETA_NODE *theta1_pointer, THETA_NODE *theta4_pointer,
		     double o06_prim[3], double o06_bis[3]);

int16_t Flange_exception(double u0[3], double v0[3], double u6[3],
	     double radius_2,
	 THETA_NODE *theta1_pointer, THETA_NODE *theta4_pointer,
	   double abd_1[3], double abd_2[3]);

THETA_NODE* Add_Theta(double Theta, double cosTheta, double sinTheta);

THETA_NODE* Create1(double x, double p, double r, double t,
		    int16_t *result,
		    double max_theta_inc, int16_t no_of_solutions,
		    double OldTheta);

void Delete_Theta_Tree(THETA_NODE *root_ptr);

void Extract_vect_from_tree(THETA_NODE *root_ptr, double Theta[5],
			    double tsin[5], double tcos[5]);

public:
  // Konstruktor.
  model_5dof (void);

  // Rozwiazanie prostego zagadnienia kinematyki.
  virtual void direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

  // Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

  // Przeliczenie polozenia koncowki zwiazane z dolaczonym narzedziem - transformacja odwrotna.
//  virtual void attached_tool_inverse_transform(lib::Homog_matrix&);

};//: kinematic_model_irp6m_5dof;


} // namespace irp6m
} // namespace kinematic
} // namespace mrrocpp


#endif

