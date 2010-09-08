#include "base/lib/mrmath/mrmath.h"

int debugi = 1;

namespace mrrocpp {
namespace lib {

/*
 ForceTrans::ForceTrans(const lib::Homog_matrix & init_frame, const lib::Homog_matrix & s_frame)
 {
 initialized = false;
 sensor_frame = s_frame;
 synchro(init_frame);
 double arm[3] = { X_AXIS_ARM , Y_AXIS_ARM , Z_AXIS_ARM };
 lib::K_vector point_of_gravity(arm);
 double weight = Z_AXIS_GRAVITY_FORCE;
 defineTool(weight, point_of_gravity);
 initialized = true;
 }
 */

ForceTrans::ForceTrans(const short l_force_sensor_name, const lib::Homog_matrix & init_frame, const lib::Homog_matrix & s_frame, const double weight, const lib::K_vector & point_of_gravity, bool _is_right_turn_frame) :
	force_sensor_name(l_force_sensor_name), initialized(false), is_right_turn_frame(true)
{

	is_right_turn_frame = _is_right_turn_frame;
	sensor_frame = s_frame;
	//	sensor_frame_translation = lib::Homog_matrix (sensor_frame.return_with_with_removed_rotation());
	// sensor_frame_translation.remove_rotation();
	//	sensor_frame_rotation  = lib::Homog_matrix (sensor_frame.return_with_with_removed_translation());
	// sensor_frame_rotation.remove_translation();
	// cout << sensor_frame;

	ft_tr_sensor_in_wrist = lib::Ft_tr(sensor_frame);

	// ft_tr_inv_sensor_translation_matrix = !ft_tr_sensor_translation_matrix;
	//	ft_tr_sensor_rotation_matrix = lib::Ft_v_tr (sensor_frame_rotation, lib::Ft_v_tr::FT);;
	// ft_tr_inv_sensor_rotation_matrix = !ft_tr_sensor_rotation_matrix;

	tool_weight = weight;
	gravity_arm_in_wrist = point_of_gravity;

	synchro(init_frame);
	defineTool(init_frame, weight, point_of_gravity);
	initialized = true;
}

void ForceTrans::defineTool(const lib::Homog_matrix & init_frame, const double weight, const lib::K_vector & point_of_gravity)
{
	tool_weight = weight;
	gravity_arm_in_wrist = point_of_gravity;
	//	gravity_force_in_base = lib::K_vector (0.0, 0.0, tool_weight);
	gravity_force_torque_in_base = lib::Ft_vector(0.0, 0.0, -tool_weight, 0.0, 0.0, 0.0);

	//	lib::frame_tab sens_rot = {{0,-1,0},{1,0,0},{0,0,1},{0,0,0}};
	//	lib::Homog_matrix sensor_rotation = lib::Homog_matrix(sens_rot);
	// orientacja koncowki manipulatora bez narzedzia
	lib::Homog_matrix current_orientation(init_frame.return_with_with_removed_translation());
	// cout << current_orientation << endl;
	// current_orientation.remove_translation(); // elminacja skladowej polozenia
	//	cout <<"aaaa"<<	endl << sensor_frame <<endl << sensor_rotation<<endl ;
	//	lib::K_vector gravity_force_in_sensor = (!(orientation*sensor_rotation))*gravity_force_in_base;
	// wyznaczenie sily grawitacji i z jej pomoca sil i momentow
	//	 lib::K_vector gravity_force_in_sensor = (!current_orientation)*gravity_force_in_base;
	lib::Ft_vector gravity_force_torque_in_sensor(lib::Ft_tr(!current_orientation) * gravity_force_torque_in_base);
	//	cout << orientation << endl;
	// wzynaczenie macierzy transformacji sil dla danego polozenia srodka ciezkosci narzedzia wzgledem czujnika
	lib::Homog_matrix tool_mass_center_translation(point_of_gravity[0], point_of_gravity[1], point_of_gravity[2]);
	ft_tool_mass_center_translation = lib::Ft_tr(tool_mass_center_translation);
	//	cout << tool_mass_center_translation << endl;

	//	reaction_torque_in_sensor = lib::K_vector((gravity_force_in_sensor*gravity_arm_in_sensor)*(-1));
	//	reaction_force_in_sensor = lib::K_vector(gravity_force_in_sensor*(-1));


	// reaction_torque_in_sensor = lib::K_vector((gravity_force_torque_in_sensor.get_force_lib::K_vector()*gravity_arm_in_sensor)*(-1));
	// reaction_force_in_sensor = lib::K_vector(gravity_force_torque_in_sensor.get_force_lib::K_vector()*(-1));

	//	cout << "aa:" << reaction_force_in_sensor << reaction_torque_in_sensor << endl;
	// wyznaczenie sil reakcji
	reaction_force_torque_in_sensor = -(ft_tool_mass_center_translation * gravity_force_torque_in_sensor);

	//	reaction_force_in_sensor = reaction_force_torque_in_sensor.get_force_lib::K_vector();
	//	reaction_torque_in_sensor = reaction_force_torque_in_sensor.get_torque_lib::K_vector();

	//	cout << "bb:" << reaction_force_torque_in_sensor << endl;
}

// zeraca sily i momenty sil w w ukladzie z orientacja koncowki manipulatory bez narzedzia
lib::Ft_vector ForceTrans::getForce(const lib::Ft_vector _inputForceTorque, const lib::Homog_matrix orientation)
{
	static long deblicz = 0;

	static int last_debugi = 0;

	lib::Ft_vector inputForceTorque = _inputForceTorque;

	if (!is_right_turn_frame) {

		inputForceTorque[2] = -inputForceTorque[2];
		inputForceTorque[5] = -inputForceTorque[5];
	}

	if (initialized) {
		deblicz++;
		/*
		 lib::K_vector input_force((double*) inputForceTorque);
		 lib::K_vector input_torque((double*) inputForceTorque+3);
		 */
		// sprowadzenie sil i momentow sil do ukladu umieszczonego w srodku czujnika ale z orientacja koncowki
		lib::Ft_vector input_force_torque(ft_tr_sensor_in_wrist * inputForceTorque);
		/*
		 if ((debugi%10==0)&&(force_sensor_name==edp::sensor::FORCE_SENSOR_ATI3084)&&(last_debugi!=debugi))
		 {
		 printf("ft: ");
		 input_force_torque.wypisz_wartosc_na_konsole();
		 }
		 */
		//		if ((deblicz%100) == 0)cout << "if" << input_force << endl;
		// by Y na podstawie kodu Slawka
		/*
		 input_force = sensor_frame_rotation*input_force;
		 input_torque = sensor_frame_rotation*input_torque;
		 */

		//		lib::K_vector input_force = input_force_torque.get_force_lib::K_vector();
		//		lib::K_vector input_torque = input_force_torque.get_torque_lib::K_vector();

		//		if ((deblicz%100) == 0)cout << "af" << input_force << endl;
		//end by Y
		//		lib::frame_tab rot = {{0,-1,0},{1,0,0},{0,0,1},{0,0,0}};
		//		lib::Homog_matrix sensor_rotation(rot);
		// orientacja koncowki manipulatora bez narzedzia
		lib::Homog_matrix current_orientation(orientation.return_with_with_removed_translation());
		//			cout << current_orientation << endl;
		// current_orientation.remove_translation (); // elminacja skladowej polozenia
		//		cout <<"bbbbb"<<	endl << sensor_frame <<endl << sensor_rotation;
		//		lib::K_vector gravity_force_in_sensor = (!(current_orientation*sensor_rotation))*gravity_force_in_base;
		// wyznaczenie sily grawitacji i z jej pomoca sil i momentow w kisci

		//		lib::K_vector gravity_force_in_sensor = (!current_orientation)*gravity_force_in_base;
		lib::Ft_vector gravity_force_torque_in_sensor(lib::Ft_tr(!current_orientation) * gravity_force_torque_in_base);

		// cout << gravity_force_in_sensor << endl;
		// uwzglednienie w odczytach sily grawitacji i sily reakcji
		//		lib::K_vector output_force = input_force - gravity_force_in_sensor - reaction_force_in_sensor;
		//		lib::K_vector output_torque = input_torque - (gravity_force_in_sensor*gravity_arm_in_sensor) - reaction_torque_in_sensor;
		//	if ((deblicz%100) == 0) cout << "aa:" << output_force << output_torque << endl;
		lib::Ft_vector output_force_torque(input_force_torque - (ft_tool_mass_center_translation
				* gravity_force_torque_in_sensor) - reaction_force_torque_in_sensor);

		//		if ((deblicz%100) == 0) cout << "bb:" << output_force_torque << endl;
		// sprowadzenie sil i momentow sil do ukladu umieszczonego w koncowce kinematyki i z jej orientacja
		// sprowadzenie sil i momentow sil do ukladu umieszczonego w koncowce kinematyki ale z orientacja ukladu bazowego
		/*
		 output_force_torque = ft_tr_sensor_translation_matrix *
		 output_force_torque;
		 */

		output_force_torque = lib::Ft_tr(current_orientation) * Ft_vector(-output_force_torque);

		//		lib::Ft_v_vector tmp_force_torque = lib::Ft_v_tr (current_orientation*sensor_frame_translation, FT_VARIANT) * output_force_torque;

		/*
		 if ((debugi%10==0)&&(force_sensor_name==edp::sensor::FORCE_SENSOR_ATI3084)&&(last_debugi!=debugi))
		 {
		 printf("ft2: ");
		 output_force_torque.wypisz_wartosc_na_konsole();
		 printf("ft3: ");
		 tmp_force_torque.wypisz_wartosc_na_konsole();
		 }
		 */

		//		output_force_torque = lib::Ft_v_tr (sensor_frame_translation) * lib::Ft_v_vector(output_force, output_torque);
		// sprowadzenie sil i momentow sil do ukladu umieszczonego w koncowce kinematyki ale z orientacja ukladu bazowego
		// output_force_torque = lib::Ft_v_tr (current_orientation) * output_force_torque;
		//

		/*
		 output_force = (current_orientation)*output_force;
		 output_torque = (current_orientation)*output_torque;
		 */
		// if ((deblicz%100) == 0) cout << output_force_torque << endl;

		/*
		 output_force.to_table(outputForceTorque);
		 //			if ((deblicz%100) == 0)cout << "of" << output_force << endl;
		 output_torque.to_table(outputForceTorque+3);
		 */
		//		lib::Ft_v_vector output_force_torque = force_from_sensor_to_tool_transform * lib::Ft_v_vector(ft_table);
		//		output_force_torque.to_table(outputForceTorque);
		/*		output_force.to_table(outputForceTorque);
		 output_torque.to_table(outputForceTorque+3);
		 input_force.to_table(outputForceTorque+6);
		 input_torque.to_table(outputForceTorque+9);
		 gravity_force_in_sensor.to_table(outputForceTorque+12);
		 (gravity_force_in_sensor*gravity_arm_in_sensor).to_table(outputForceTorque+15);
		 reaction_force_in_sensor.to_table(outputForceTorque+18);
		 reaction_torque_in_sensor.to_table(outputForceTorque+21);
		 for(int j=0;j<4;j++)
		 for(int i=j*6;i<j*6+3;i++) {
		 outputForceTorque[i]*=20;
		 outputForceTorque[i+3]*=333;
		 }

		 */
		last_debugi = debugi;
		return output_force_torque;
	}
	return 0;
}

void ForceTrans::synchro(const lib::Homog_matrix & init_frame)
{
	//initialisation_frame = init_frame;
	if (initialized)
		defineTool(init_frame, tool_weight, gravity_arm_in_wrist);
}

} // namespace lib
} // namespace mrrocpp
