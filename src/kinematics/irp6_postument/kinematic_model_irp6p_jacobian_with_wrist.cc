// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6p_jacobian_with_wrist.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- deklaracja klasy
//				- przedefiniowanie rozwiazania odwrotnego zadania
//				  kinematyki - metoda uwzgledniajaca jakobian
//
// Autor:		Anna Maria Sibilska
// Data:		18.07.2007
// ------------------------------------------------------------------------

// Klasa kinematic_model_calibrated_irp6p_with_wrist.
#include "kinematics/irp6_postument/kinematic_model_irp6p_jacobian_with_wrist.h"


/* -----------------------------------------------------------------------
  Konstruktor.
 ------------------------------------------------------------------------- */
kinematic_model_irp6p_jacobian_with_wrist::kinematic_model_irp6p_jacobian_with_wrist (void)
{
  // Ustawienie etykiety modelu kinematycznego.
  set_kinematic_model_label("Switching to kinematic based on jacobian matrix");

   // Przeliczac do globalnego ukladu odniesienia.
  global_frame_computations =false;
  // Wykonywac przeliczenia zwiazane z narzedziem.
  attached_tool_computations = true;

}; //konstruktor

/* ------------------------------------------------------------------------
  Zadanie odwrotne kinematyki dla robota IRp-6 na postumencie w oparciu o odwrotnosc jakobianu

  Wejscie:
  * local_current_joints - obecne (w rzeczywistosci poprzednie) wspolrzedne wewnetrzne robota (kolejno q0, q1, q2, ...)
  * local_desired_end_effector_frame - macierz przeksztacenia jednorodnego (MPJ)
		opisujca zadane poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.

  Wyjscie:
  * local_desired_joints - wyliczone wspolrzedne wewnetrzne robota (kolejno q0, q1, q2, ...)
 ------------------------------------------------------------------------ */

void kinematic_model_irp6p_jacobian_with_wrist::inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, frame_tab* local_desired_end_effector_frame)
{

  double K=1; 			//Zadane wzmocnienie
  double E=0.00001;		//Uchyb dla kt�rego rozwiazanie zaakceptowane
  double Max;			//pomocnbicza zmienna - max element wektora

  frame_tab local_current_end_effector_frame;	//Ramka odpowiadajaca aktualnej pozycji
  Ft_v_vector desired_distance_new;				//odleglosc do pokonania
  Ft_v_vector delta_q;									//przyrost zmieenych przegubowych na jedna iteracje
  Ft_v_vector current_joints;							//wartosci aktualnych zmiennych przegubowych reprezentowane jako wektor
  Jacobian_matrix  jacobian_new;					//jakobian

  current_joints.set_values(local_current_joints);
  attached_tool_computations = false;

  //wyliczenie prostego zadania kinematyki dla aktualnej konfiguracji
  i2e_transform(local_current_joints, &local_current_end_effector_frame);
  attached_tool_computations = true;
  desired_distance_new.position_distance(&local_current_end_effector_frame, local_desired_end_effector_frame);

 // printf("jacobian inverse\n");

   //Wyliczenie max elementu uchybu
   Max=desired_distance_new.max_element();

   //Wzmocnienie
   desired_distance_new=desired_distance_new*K;

while(fabs(Max)>E){

	//Wzory na jakobian dla Irp-6 o 6 stopniach swobody
     jacobian_new.irp6_6dof_equations(current_joints);

	//Wyliczenie przyrostu zmiennych przegubowych
     delta_q= jacobian_new.jacobian_inverse_gauss(desired_distance_new);
	current_joints=current_joints+delta_q;
	current_joints.to_table(local_current_joints);

    //wyliczenie prostego zadania kinematyki dla nowo wyliczonej konfiguracji
	attached_tool_computations = false;
	i2e_transform(local_current_joints, &local_current_end_effector_frame);
	attached_tool_computations = true;

	//Wyliczenie uchybu pozycji dla zadanej tymczasowej i porzadanej ramki
	desired_distance_new.position_distance(&local_current_end_effector_frame, local_desired_end_effector_frame);

	Max=desired_distance_new.max_element();
	desired_distance_new=desired_distance_new*K;
}

  for (int i=0; i<=5; i++){
		local_desired_joints[i]=local_current_joints[i];}

  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
 check_joints (local_desired_joints);

}; //: inverse_kinematics_transform()
