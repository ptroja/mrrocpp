// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6ot_with_track.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na torze
//				- definicja metod klasy
//				- wykorzystanie toru jako czynnego stopnia swobody
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

// Klasa kinematic_model_irp6ot_with_track.
#include "kinematics/irp6_on_track/kinematic_model_irp6ot_with_track.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot {

/* -----------------------------------------------------------------------
  Konstruktor.
 ------------------------------------------------------------------------- */
model_with_track::model_with_track (int _number_of_servos):
	model_with_wrist(_number_of_servos)
{
  // Ustawienie etykiety modelu kinematycznego.
  set_kinematic_model_label("Switching to kinematic model with active track");

  // Ustawienie parametrow kinematycznych - przez konstruktor with_wrist.
} // end: kinematic_model::kinematic_model


/* ------------------------------------------------------------------------
  Zadanie proste kinematyki dla robota IRp-6 na torze.
  Wykorzystanie toru jako czynnego stopnia swobody.

  Wejscie:
  * current_joints[6] - wspolrzedne wewnetrzne robota (kolejno d0, q1, q2, ...)

  Wyjscie:
  * current_end_effector_frame[4][3] - macierz przeksztacenia jednorodnego (MPJ)
		opisujca aktualne poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.
 ------------------------------------------------------------------------ */
void model_with_track::direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame) {

  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
  check_joints (local_current_joints);

  // Parametry pomocnicze.
  double d0 = local_current_joints[0];
  double s1 = sin(local_current_joints[1]);
  double c1 = cos(local_current_joints[1]);
  double s2 = sin(local_current_joints[2]);
  double c2 = cos(local_current_joints[2]);
  double s3 = sin(local_current_joints[3]);
  double c3 = cos(local_current_joints[3]);
  double s4 = sin(local_current_joints[4]);
  double c4 = cos(local_current_joints[4]);
  double s5 = sin(local_current_joints[5]);
  double c5 = cos(local_current_joints[5]);

  // Proste zadanie kinematyki.
  local_current_end_effector_frame(0,0) = c5*s4*c1+s1*s5;
  local_current_end_effector_frame(0,1) = -s5*s4*c1+s1*c5;
  local_current_end_effector_frame(0,2) = c4*c1;
  local_current_end_effector_frame(0,3) = d6*c4*c1 + a3*c3*c1 + a2*c1*c2;
  local_current_end_effector_frame(1,0) = c5*s4*s1-c1*s5;
  local_current_end_effector_frame(1,1) = -s5*s4*s1-c1*c5;
  local_current_end_effector_frame(1,2) = c4*s1;
  local_current_end_effector_frame(1,3) = d6*c4*s1 + a3*c3*s1 + a2*s1*c2 + d0;
  local_current_end_effector_frame(2,0) = c4*c5;
  local_current_end_effector_frame(2,1) = -c4*s5;
  local_current_end_effector_frame(2,2) = -s4;
  local_current_end_effector_frame(2,3) = -d6*s4 - a3*s3 - a2*s2 + d1;

} //:: direct_kinematics_transform()

/* ------------------------------------------------------------------------
  Zadanie odwrotne kinematyki dla robota IRp-6 na torze.
  Wykorzystanie toru jako czynnego stopnia swobody.

  Wejscie:
  * local_current_joints - obecne (w rzeczywistosci poprzednie) wspolrzedne wewnetrzne robota (kolejno d0, q1, q2, ...)
  * local_desired_end_effector_frame - macierz przeksztacenia jednorodnego (MPJ)
		opisujca zadane poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.

  Wyjscie:
  * local_desired_joints - wyliczone wspolrzedne wewnetrzne robota (kolejno d0, q1, q2, ...)
 ------------------------------------------------------------------------ */

void model_with_track::inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// pierwsze przyisanie bo ktos to kiedys sk......

	for (int i=0; i<=6; i++)
	{
		local_desired_joints[i] = local_current_joints[i];
	}



  // Stale
  const double a2_2 = a2*a2;
  const double a3_2 = a3*a3;
  const double EPS=1e-10;
  // Zmienne pomocnicze.

// Sprawdzenie przypadkow osobliwych:
bool p=false;
// We wszystkich przypadkach przyjmuje sie ,,stare'' d0==theta[0]
// 1a) Kisc zwrocona do pionowo gory, theta4=-pi/2
  if ( local_desired_end_effector_frame(2,2) > 0 && ABS(local_desired_end_effector_frame(2,0)) <= EPS && ABS(local_desired_end_effector_frame(2,1)) <= EPS ) {
    std::cout << " Osobliwosc! (theta4=-pi/2)" << std::endl;
    p=true;
    local_desired_joints[1] = M_PI-atan2( local_desired_end_effector_frame(1,3)-local_desired_joints[0], -local_desired_end_effector_frame(0,3) );
    local_desired_joints[5] = atan2( -local_desired_end_effector_frame(1,0), -local_desired_end_effector_frame(0,0) )-local_desired_joints[1];
    local_desired_joints[4] = -M_PI_2;
  }

// 1b) Kisc zwrocona do pionowo do dolu, theta4= pi/2
  if ( local_desired_end_effector_frame(2,2) < 0 && ABS(local_desired_end_effector_frame(2,0))<=EPS && ABS(local_desired_end_effector_frame(2,1))<=EPS ){
    std::cout << " Osobliwosc! (theta4=pi/2)" << std::endl;
    p=true;
    local_desired_joints[1]=M_PI-atan2( local_desired_end_effector_frame(1,3)-local_desired_joints[0], -local_desired_end_effector_frame(0,3) );
    local_desired_joints[5]=local_desired_joints[1]-atan2(  local_desired_end_effector_frame(1,0),  local_desired_end_effector_frame(0,0) );
    local_desired_joints[4]=M_PI_2;
  }

// 2a) Ramie robota w osi toru -- theta[1]=pi/2
  if ( !p && ABS(local_desired_end_effector_frame(0,3))<=EPS && local_desired_end_effector_frame(1,2)>0){
    std::cout << " Osobliwosc! (theta1=pi/2)" << std::endl;
    local_desired_joints[5] = atan2 ( local_desired_end_effector_frame(0,0), local_desired_end_effector_frame(0,1) );
    local_desired_joints[4] = atan2 ( -local_desired_end_effector_frame(2,2), local_desired_end_effector_frame(1,2) );
    local_desired_joints[1] = M_PI_2;
    p=true;
  }

// 2b) Ramie robota w osi toru --  theta[1]=-pi/2
  if ( !p && ABS(local_desired_end_effector_frame(0,3))<=EPS && local_desired_end_effector_frame(1,2)<0){
    std::cout << " Osobliwosc! (theta1=-pi/2)" << std::endl;
    local_desired_joints[5] = atan2 ( -local_desired_end_effector_frame(0,0), -local_desired_end_effector_frame(0,1) );
    local_desired_joints[4] = atan2 ( -local_desired_end_effector_frame(2,2), -local_desired_end_effector_frame(1,2) );
    local_desired_joints[1] = -M_PI_2;
    p=true;
  }

// Przypadek nieosobliwy:
if (!p) {
  local_desired_joints[1] = atan2(  local_desired_end_effector_frame(1,2), local_desired_end_effector_frame(0,2) );
  local_desired_joints[4] = atan2( -local_desired_end_effector_frame(2,2), local_desired_end_effector_frame(0,2)/cos(local_desired_joints[1]) );
  local_desired_joints[5] = atan2( -local_desired_end_effector_frame(2,1), local_desired_end_effector_frame(2,0) );
  local_desired_joints[0] = -local_desired_end_effector_frame(0,3)*sin(local_desired_joints[1])/cos(local_desired_joints[1])+
	local_desired_end_effector_frame(1,3);
}

  // Obliczanie katow theta[2] oraz theta[3] -- podejscie geometryczne
  // Zmienne pomocnicze - px1^2+py1^2+pz1^2 = dlugosc odcinka laczacego
  // koniec ramienia gornego (a3) z poczatkiem globalnego ukladu wspolrzednych
  double c4 = cos(local_desired_joints[4]);
  double s4 = sin(local_desired_joints[4]);
  double px1 = local_desired_end_effector_frame(0,3) - d6*c4*cos(local_desired_joints[1]);
  double py1 = local_desired_end_effector_frame(1,3) - d6*c4*sin(local_desired_joints[1])-local_desired_joints[0];
  double pz1 = local_desired_end_effector_frame(2,3) + d6*s4-d1;
  double d = sqrt(px1*px1+py1*py1+pz1*pz1);
  double alfa = asin(pz1/d);

  local_desired_joints[2] = -alfa - acos((a2_2+d*d-a3_2)/(2*a2*d));
  local_desired_joints[3] = asin((-d6*s4-a2*sin(local_desired_joints[2])+d1 - local_desired_end_effector_frame(2,3) )/a3);

//  local_desired_joints[6] = local_current_joints[6];


  // Sprowadzenie katow do I lub II cwiartki ukladu
  for (int i=1; i<6; i++ ) {
    lib::reduce(local_desired_joints[i]);
  }

  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
  check_joints (local_desired_joints);

} //: inverse_kinematics_transform()


} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp

