/*!
 * @file
 * @brief File containing methods of the IRp-6p with wrist (6DOF) jacobian based kinematic model class.
 *
 * @author Anna Maria Sibilska
 * @author tkornuta
 * @date 18.07.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS irp6p_m
 */

#include "robot/irp6p_m/kinematic_model_irp6p_jacobian_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6p {

model_jacobian_with_wrist::model_jacobian_with_wrist (int _number_of_servos):
	model_with_wrist(_number_of_servos)
{
  // Ustawienie etykiety modelu kinematycznego.
  set_kinematic_model_label("Switching to kinematic based on jacobian matrix");

   // Przeliczac do globalnego ukladu odniesienia.
  global_frame_computations =false;
  // Wykonywac przeliczenia zwiazane z narzedziem.
  attached_tool_computations = true;

}


void model_jacobian_with_wrist::inverse_kinematics_transform(lib::JointArray & local_desired_joints, lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{

  double K=1; 			//Zadane wzmocnienie
  double E=0.00001;		//Uchyb dla ktorego rozwiazanie zaakceptowane
  double Max;			//pomocnbicza zmienna - max element wektora

  lib::Homog_matrix local_current_end_effector_frame;	//Ramka odpowiadajaca aktualnej pozycji
  lib::Xyz_Angle_Axis_vector desired_distance_new;				//odleglosc do pokonania
  lib::Xyz_Angle_Axis_vector delta_q;									//przyrost zmieenych przegubowych na jedna iteracje
  lib::Xyz_Angle_Axis_vector current_joints;							//wartosci aktualnych zmiennych przegubowych reprezentowane jako wektor
  lib::Jacobian_matrix  jacobian_new;					//jakobian

  current_joints.set_values(&local_current_joints[0]);
  attached_tool_computations = false;

  //wyliczenie prostego zadania kinematyki dla aktualnej konfiguracji
  i2e_transform(local_current_joints, local_current_end_effector_frame);
  attached_tool_computations = true;
  desired_distance_new.position_distance(local_current_end_effector_frame, local_desired_end_effector_frame);

 // printf("jacobian inverse\n");

   //Wyliczenie max elementu uchybu
   Max=desired_distance_new.max_element();

   //Wzmocnienie
   desired_distance_new *= K;

while(fabs(Max)>E){

	//Wzory na jakobian dla Irp-6 o 6 stopniach swobody
     jacobian_new.irp6_6dof_equations(current_joints);

	//Wyliczenie przyrostu zmiennych przegubowych
     delta_q= jacobian_new.jacobian_inverse_gauss(desired_distance_new);
	current_joints += delta_q;
	current_joints.to_table(&local_current_joints[0]);

    //wyliczenie prostego zadania kinematyki dla nowo wyliczonej konfiguracji
	attached_tool_computations = false;
	i2e_transform(local_current_joints, local_current_end_effector_frame);
	attached_tool_computations = true;

	//Wyliczenie uchybu pozycji dla zadanej tymczasowej i porzadanej ramki
	desired_distance_new.position_distance(local_current_end_effector_frame, local_desired_end_effector_frame);

	Max=desired_distance_new.max_element();
	desired_distance_new *= K;
}

  for (int i=0; i<=5; i++){
		local_desired_joints[i]=local_current_joints[i];}

  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
 check_joints (local_desired_joints);

} //: inverse_kinematics_transform()

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp

