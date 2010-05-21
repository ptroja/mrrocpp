/*
 * bang_bang_profile.cpp
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#include "bang_bang_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

bang_bang_profile::bang_bang_profile() {
	// TODO Auto-generated constructor stub

}

bang_bang_profile::~bang_bang_profile() {
	// TODO Auto-generated destructor stub
}

bool bang_bang_profile::eq(double a, double b) {
	const double EPS = 0.0001;
	const double& diff = a - b;
	return diff < EPS && diff > -EPS;
}

bool bang_bang_profile::reduction_model_1(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s) {

	//printf("redukcja kinematic_model_with_tool 1 w osi: %d\n", i);
	if (pose_vector_iterator->v_p[i] < pose_vector_iterator->v_k[i] && (pose_vector_iterator->v_k[i] * pose_vector_iterator->t
			- 0.5 * ((pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]) *
					(pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]))/pose_vector_iterator->a_r[i]) > s) {//proba dopasowania do modelu 2

		pose_vector_iterator->model[i] = 2;
		//printf("dopasowanie kinematic_model_with_tool 2 w redukcji modelu 1\n");
		reduction_model_2(pose_vector_iterator, i, s);

	} else if (pose_vector_iterator->v_p[i] > pose_vector_iterator->v_k[i] && (pose_vector_iterator->v_p[i] * pose_vector_iterator->t
			- 0.5 * ((pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]) *
					(pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]))/pose_vector_iterator->a_r[i]) > s){ // proba dopasowania do modelu 4

		pose_vector_iterator->model[i] = 4;
		//printf("dopasowanie kinematic_model_with_tool 4 w redukcji modelu 1\n");
		reduction_model_4(pose_vector_iterator, i, s);

	} else if (eq(pose_vector_iterator->v_p[i], pose_vector_iterator->v_k[i]) && pose_vector_iterator->v_k[i] * pose_vector_iterator->t > s) {//proba dopasowanie do modelu 3
		//printf("dopasowanie kinematic_model_with_tool 3 w redukcji modelu 1\n");
		reduction_model_3(pose_vector_iterator, i, s);

	} else { //normalna redukcja dla modelu 1
		//printf("normalna redukcja kinematic_model_with_tool 1\n");

		double t1;//czas przyspieszania
		double t2;//czas jednostajnego
		double delta;//delta w rownaniu kwadratowym

		delta = (2 * pose_vector_iterator->a_r[i] * pose_vector_iterator->t + 2 * pose_vector_iterator->v_k[i] + 2 * pose_vector_iterator->v_p[i]) *
				(2 * pose_vector_iterator->a_r[i] * pose_vector_iterator->t + 2 * pose_vector_iterator->v_k[i] + 2 * pose_vector_iterator->v_p[i]) +
				8 * (- pose_vector_iterator->v_p[i] * pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i] * pose_vector_iterator->v_k[i] -
				2 * pose_vector_iterator->a_r[i] * s);

		//printf("delta: %f\n", delta);

		//printf("t: %f\t a_r: %f\t v_p: %f\n",pose_vector_iterator->t, pose_vector_iterator->a_r[i],pose_vector_iterator->v_p[i]);

		if (!eq(delta,0.0) && !eq(delta,-0.0)) {
			pose_vector_iterator->v_r[i] = (-(2 * pose_vector_iterator->a_r[i] * pose_vector_iterator->t + 2 * pose_vector_iterator->v_k[i] +
				                       2 * pose_vector_iterator->v_p[i]) + sqrt(delta)) / (-4);
		} else {
			pose_vector_iterator->v_r[i] = (-(2 * pose_vector_iterator->a_r[i] * pose_vector_iterator->t + 2 * pose_vector_iterator->v_k[i] +
							                       2 * pose_vector_iterator->v_p[i])) / (-4);
		}

		printf("v_r: %f\n", pose_vector_iterator->v_r[i]);
		t1 = fabs(pose_vector_iterator->v_p[i] - pose_vector_iterator->v_r[i]) / pose_vector_iterator->a_r[i];
		t2 = pose_vector_iterator->t - t1 - (fabs(pose_vector_iterator->v_k[i] - pose_vector_iterator->v_r[i]) / pose_vector_iterator->a_r[i]);

		//printf("t2: %f\t t1: %f\n", t2, t1);

		pose_vector_iterator->s_przysp[i] = t1 * pose_vector_iterator->v_p[i] +
										  0.5 * pose_vector_iterator->a_r[i] * t1 * t1;
		pose_vector_iterator->s_jedn[i] = pose_vector_iterator->v_r[i] * t2;
	}
}

bool bang_bang_profile::reduction_model_2(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s) {
	double a;

	a = (0.5 * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]) * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i])
				+ pose_vector_iterator->v_p[i] * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i])
				- pose_vector_iterator->v_k[i] * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]) )
				/ (s - pose_vector_iterator->v_k[i] * pose_vector_iterator->t);

	if ((pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]) / a > pose_vector_iterator->t) { //drugi stopien redukcji
		if (s == pose_vector_iterator->v_p[i] * pose_vector_iterator->t) { //zabezpieczenie przed dzieleniem przez 0
			a = -1; //powoduje przejscie do nastepnego stopnia redukcji (moglaby byc dowolna ujemna liczba)
		} else {
			a = (0.5 * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]) * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i])) /
				(s - pose_vector_iterator->v_p[i]*pose_vector_iterator->t);
		}

		if (a > pose_vector_iterator->a_r[i] || a <= 0) {//trzeci stopien redukcji

			double t1; //czas konca opoznienia
			double s1; // droga w etapie w ktorym redukujemy czas (przed etapem "odcinanym")
			double t2; //czas redukowanego kawalka

			t2 = pose_vector_iterator->t - ((pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]) / pose_vector_iterator->a_r[i]);

			s1 = s - (pose_vector_iterator->v_p[i] * (pose_vector_iterator->t- t2)
			   + 0.5 * pose_vector_iterator->a_r[i] * (pose_vector_iterator->t - t2) * (pose_vector_iterator->t - t2));

			if (pose_vector_iterator->a_r[i] * pose_vector_iterator->a_r[i] * t2 * t2 //ujemna liczba pod pierwiastkiem, zabezpieczenie
					- 4 * pose_vector_iterator->a_r[i] * (pose_vector_iterator->v_p[i] * t2 - s1) < 0) {
				return vp_reduction(pose_vector_iterator, i, s, pose_vector_iterator->t);
			}

			t1 = (pose_vector_iterator->a_r[i] * t2
					- (sqrt(pose_vector_iterator->a_r[i] * pose_vector_iterator->a_r[i] * t2 * t2
					- 4 * pose_vector_iterator->a_r[i] * (pose_vector_iterator->v_p[i] * t2 - s1))) )
					/ (2 * pose_vector_iterator->a_r[i]);

			if (pose_vector_iterator->v_p[i] - pose_vector_iterator->a_r[i] * t1 < 0) {//ujemna predkosc ruchu, zabezpieczenie
				return vp_reduction(pose_vector_iterator, i, s, pose_vector_iterator->t);
			}

			pose_vector_iterator->v_r[i] = pose_vector_iterator->v_p[i] - pose_vector_iterator->a_r[i] * t1;
			pose_vector_iterator->s_przysp[i] = 0.5 * pose_vector_iterator->a_r[i] * t1 * t1 + pose_vector_iterator->v_r[i] * t1;
			pose_vector_iterator->s_jedn[i] = pose_vector_iterator->v_r[i] * (t2 - 2 * t1);



			return true;
		}

		pose_vector_iterator->a_r[i] = a;
		pose_vector_iterator->v_r[i] = pose_vector_iterator->v_p[i];
		pose_vector_iterator->s_przysp[i] = 0;
		pose_vector_iterator->s_jedn[i] = pose_vector_iterator->v_r[i] * (pose_vector_iterator->t -
										(pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i]);

		return true;
	}

	pose_vector_iterator->a_r[i] = a;
	//printf("zredukowane a_r: %f\n", pose_vector_iterator->a_r[i]);
	pose_vector_iterator->v_r[i] = pose_vector_iterator->v_k[i];
	pose_vector_iterator->s_przysp[i] = pose_vector_iterator->v_p[i] * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i]
	                                  + 0.5 * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i])
	                                  * (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]) / pose_vector_iterator->a_r[i];
	pose_vector_iterator->s_jedn[i] = pose_vector_iterator->v_r[i] * (pose_vector_iterator->t - (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i]) / pose_vector_iterator->a_r[i]);

	return true;
}

bool bang_bang_profile::reduction_model_3(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s) {
	double t1; //czas konca opoznienia

	if(pose_vector_iterator->a_r[i] * pose_vector_iterator->a_r[i] * pose_vector_iterator->t * pose_vector_iterator->t//liczba pierwiastkowana mniejsza od 0, zabezpieczenie
			- 4 * pose_vector_iterator->a_r[i] * (pose_vector_iterator->v_p[i] * pose_vector_iterator->t - s) < 0) {
		return vp_reduction(pose_vector_iterator, i, s, pose_vector_iterator->t);
	}

	t1 = (pose_vector_iterator->a_r[i] * pose_vector_iterator->t
			- (sqrt(pose_vector_iterator->a_r[i] * pose_vector_iterator->a_r[i] * pose_vector_iterator->t * pose_vector_iterator->t
			- 4 * pose_vector_iterator->a_r[i] * (pose_vector_iterator->v_p[i] * pose_vector_iterator->t - s))) )
			/ (2 * pose_vector_iterator->a_r[i]);

	if (pose_vector_iterator->v_p[i] - pose_vector_iterator->a_r[i] * t1 < 0) {//ujemna predkosc ruchu, zabezpieczenie
		return vp_reduction(pose_vector_iterator, i, s, pose_vector_iterator->t);
	}

	pose_vector_iterator->v_r[i] = pose_vector_iterator->v_p[i] - pose_vector_iterator->a_r[i] * t1;
	pose_vector_iterator->s_przysp[i] = 0.5 * pose_vector_iterator->a_r[i] * t1 * t1 + pose_vector_iterator->v_r[i] * t1;
	pose_vector_iterator->s_jedn[i] = pose_vector_iterator->v_r[i] * (pose_vector_iterator->t - 2 * t1);

	return true;
}

bool bang_bang_profile::reduction_model_4(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s) {
	double a;

	a = (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]) * (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]) /
		((-2) * (s - pose_vector_iterator->v_p[i] * pose_vector_iterator->t));

	if ((pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]) / a > pose_vector_iterator->t) { //drugi stopien redukcji
		if (s == pose_vector_iterator->v_k[i] * pose_vector_iterator->t) { //zabezpieczenie przed dzieleniem przez 0
			a = -1; //powoduje przejscie do nastepnego stopnia redukcji (moglaby byc dowolna ujemna liczba)
		} else {
			a = (0.5 * (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]) * (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i])) /
				(s - pose_vector_iterator->v_k[i]*pose_vector_iterator->t);
		}
		if (a > pose_vector_iterator->a_r[i] || a <= 0) {//trzeci stopien redukcji
			double t1; //czas konca opoznienia (relatywny - liczac od poczatku redukowanego odcinka)
			double s1; // droga w etapie w ktorym redukujemy czas (po etapie odcinanym)
			double t2; //czas redukowanego kawalka (relatywny)

			t2 = pose_vector_iterator->t - ((pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]) / pose_vector_iterator->a_r[i]);

			s1 = s - (pose_vector_iterator->v_k[i] * (pose_vector_iterator->t - t2)
			   + 0.5 * pose_vector_iterator->a_r[i] *(pose_vector_iterator->t - t2) * (pose_vector_iterator->t - t2));

			if (pose_vector_iterator->a_r[i] * pose_vector_iterator->a_r[i] * t2 * t2 //liczba pod pierwiastkiem mniejsza od 0, zabezpieczenie
				- 4 * pose_vector_iterator->a_r[i] * (pose_vector_iterator->v_k[i] * t2 - s1) < 0) {
				return vp_reduction(pose_vector_iterator, i, s, pose_vector_iterator->t);
			}

			t1 = (pose_vector_iterator->a_r[i] * t2
					- (sqrt(pose_vector_iterator->a_r[i] * pose_vector_iterator->a_r[i] * t2 * t2
					- 4 * pose_vector_iterator->a_r[i] * (pose_vector_iterator->v_k[i] * t2 - s1))) )
					/ (2 * pose_vector_iterator->a_r[i]);

			if ((pose_vector_iterator->v_k[i] - pose_vector_iterator->a_r[i] * t1) < 0) {//ujemna predkosc ruchu, zabezpieczenie
				return vp_reduction(pose_vector_iterator, i, s, pose_vector_iterator->t);
			}

			pose_vector_iterator->v_r[i] = pose_vector_iterator->v_k[i] - pose_vector_iterator->a_r[i] * t1;
			pose_vector_iterator->s_przysp[i] = 0.5 * pose_vector_iterator->a_r[i] * t1 * t1 + pose_vector_iterator->v_r[i] * t1
											+ pose_vector_iterator->v_k[i] * (pose_vector_iterator->t - t2)
											+ 0.5 * pose_vector_iterator->a_r[i] * (pose_vector_iterator->t - t2) * (pose_vector_iterator->t - t2);
			pose_vector_iterator->s_jedn[i] = pose_vector_iterator->v_r[i] * (t2 - 2 * t1);

			return true;
		}

		pose_vector_iterator->a_r[i] = a;
		//printf("a_r: %f", pose_vector_iterator->v_p[i]);
		pose_vector_iterator->v_r[i] = pose_vector_iterator->v_k[i];
		pose_vector_iterator->s_przysp[i] = pose_vector_iterator->v_r[i] * (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i])/pose_vector_iterator->a_r[i]
		                                + 0.5 * (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]) * (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i])
		                                /pose_vector_iterator->a_r[i];
		pose_vector_iterator->s_jedn[i] = pose_vector_iterator->v_r[i] * (pose_vector_iterator->t
									  - (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i])/pose_vector_iterator->a_r[i]);

		return true;
	}

	pose_vector_iterator->a_r[i] = a;
	pose_vector_iterator->v_r[i] = pose_vector_iterator->v_p[i];
	pose_vector_iterator->s_przysp[i] = 0;
	pose_vector_iterator->s_jedn[i] = pose_vector_iterator->v_p[i] * (pose_vector_iterator->t - (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i]) / pose_vector_iterator->a_r[i]);

	return true;
}

bool bang_bang_profile::vp_reduction(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s, double t) {
	double v_r; //zmiana ruchu na jednostajny

	v_r = s/t;

	pose_vector_iterator->v[i] = v_r/pose_vector_iterator->v_max[i];
	/*switch (td.arm_type) {//zapisanie nowej predkoscici w liscie pozycji, dla danej pozycji

		case lib::ECP_XYZ_EULER_ZYZ:
			pose_list_backup_iterator->v[i] = v_r/v_max_zyz[i];
			break;

		case lib::ECP_XYZ_ANGLE_AXIS:
			pose_list_backup_iterator->v[i] = v_r/v_max_aa[i];
			break;

		case lib::ECP_MOTOR:
			pose_list_backup_iterator->v[i] = v_r/v_max_motor[i];
			break;

		case lib::ECP_JOINT:
			pose_list_backup_iterator->v[i] = v_r/v_max_zyz[i];
			break;
		default:
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}

	return v_r;*/
	//pose_list = pose_list_backup;

	//calculate();
	return false;
}

bool bang_bang_profile::vk_reduction(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s, double t) {
	//printf("v_k redukcja w osi: %d\n", i);
	double a;
	double v_k;

	a = (2 * (s - (pose_vector_iterator->v_p[i] * t))) / (t * t);

	if (a < 0) {
		//printf("v_k stopien 2\n");
		a = (-2 * s + 2 * t * pose_vector_iterator->v_p[i]) / (t * t);
		v_k = (-1) * a * t + pose_vector_iterator->v_p[i];
		if (eq(v_k, -0.0)) {//glupie... ale inaczej nie dziala gdy v_k jest rowne 0
			v_k = 0.0;
		}

		if (a > pose_vector_iterator->a_r[i] || v_k < 0 || v_k > pose_vector_iterator->v_k[i]) {
			//printf("v_k: %f\n", v_k);
			return vp_reduction(pose_vector_iterator, i, s, t);
		}

		pose_vector_iterator->v_k[i] = v_k;
		pose_vector_iterator->a_r[i] = a;
		pose_vector_iterator->v_r[i] = pose_vector_iterator->v_p[i];
		pose_vector_iterator->s_przysp[i] = 0;
		pose_vector_iterator->s_jedn[i] = 0;
		return true;
	}

	pose_vector_iterator->v_k[i] = a * t + pose_vector_iterator->v_p[i];
	pose_vector_iterator->a_r[i] = a;
	pose_vector_iterator->v_r[i] = pose_vector_iterator->v_k[i];
	pose_vector_iterator->s_przysp[i] = s;
	pose_vector_iterator->s_jedn[i] = 0;

	return true;
}

bool bang_bang_profile::optimize_time1(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s) {
	double v_r;
	double t;

	v_r = sqrt(pose_vector_iterator->a_r[i] * s + (pose_vector_iterator->v_p[i] * pose_vector_iterator->v_p[i])/2 + (pose_vector_iterator->v_k[i] * pose_vector_iterator->v_k[i])/2);
	//pose_vector_iterator->t = t;

	if (pose_vector_iterator->v_p[i] >= pose_vector_iterator->v_k[i] && v_r < pose_vector_iterator->v_p[i]) {
		t = (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i])/pose_vector_iterator->a_r[i];
		return vp_reduction(pose_vector_iterator, i, s, t);
	} else if (pose_vector_iterator->v_p[i] < pose_vector_iterator->v_k[i] && v_r < pose_vector_iterator->v_k[i]) {
		t = (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i];
		pose_vector_iterator->v_r[i] = pose_vector_iterator->v_k[i];
		return vk_reduction(pose_vector_iterator, i, s, t);
	} else {
		t = ((v_r - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i] + (v_r - pose_vector_iterator->v_k[i])/pose_vector_iterator->a_r[i]);
		pose_vector_iterator->v_r[i] = v_r;
	}
	pose_vector_iterator->t = t;
}

bool bang_bang_profile::optimize_time2(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s) {
	double v_r;
	double t;

	v_r = sqrt(2 * pose_vector_iterator->a_r[i] * s + pose_vector_iterator->v_p[i] * pose_vector_iterator->v_p[i]);
	pose_vector_iterator->v_k[i] = v_r;
	t = (pose_vector_iterator->v_k[i] - pose_vector_iterator->v_p[i])/pose_vector_iterator->a_r[i];

	pose_vector_iterator->t = t;

	return true;
}

bool bang_bang_profile::optimize_time4(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &pose_vector_iterator, int i, double s) {
	double v_r;
	double t;

	v_r = sqrt(2 * pose_vector_iterator->a_r[i] * s + pose_vector_iterator->v_k[i] * pose_vector_iterator->v_k[i]);
	pose_vector_iterator->v_p[i] = v_r; //niepotrzebne?
	t = (pose_vector_iterator->v_p[i] - pose_vector_iterator->v_k[i])/pose_vector_iterator->a_r[i];

	pose_vector_iterator->t = t;

	return true;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
