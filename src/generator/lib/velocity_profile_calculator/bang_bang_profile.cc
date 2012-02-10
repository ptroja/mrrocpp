/**
 * @file
 * @brief Contains definitions of the methods of bang_bang_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#include <cstdio>

#include "bang_bang_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

using namespace std;

bang_bang_profile::bang_bang_profile() {
	// TODO Auto-generated constructor stub

}

bang_bang_profile::~bang_bang_profile() {
	// TODO Auto-generated destructor stub
}

bool bang_bang_profile::reduction_model_1(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {

	//printf("redukcja kinematic_model_with_tool 1 w osi: %d\n", i);
	if (it->v_p[i] < it->v_k[i] && (it->v_k[i] * it->times[i]
			- 0.5 * ((it->v_k[i] - it->v_p[i]) *
					(it->v_k[i] - it->v_p[i]))/it->a_r[i]) > it->s[i]) {//proba dopasowania do modelu 2

		it->model[i] = 2;
		//printf("dopasowanie kinematic_model_with_tool 2 w redukcji modelu 1\n");
		reduction_model_2(it, i);

	} else if (it->v_p[i] > it->v_k[i] && (it->v_p[i] * it->times[i]
			- 0.5 * ((it->v_p[i] - it->v_k[i]) *
					(it->v_p[i] - it->v_k[i]))/it->a_r[i]) > it->s[i]){ // proba dopasowania do modelu 4

		it->model[i] = 4;
		//printf("dopasowanie kinematic_model_with_tool 4 w redukcji modelu 1\n");
		reduction_model_4(it, i);

	} else if (eq(it->v_p[i], it->v_k[i]) && it->v_k[i] * it->times[i] > it->s[i]) {//proba dopasowanie do modelu 3
		//printf("dopasowanie kinematic_model_with_tool 3 w redukcji modelu 1\n");
		reduction_model_3(it, i);

	} else { //normalna redukcja dla modelu 1
		//printf("normalna redukcja kinematic_model_with_tool 1\n");

		double t1;//czas przyspieszania
		double t2;//czas jednostajnego
		double delta;//delta w rownaniu kwadratowym

		delta = (2 * it->a_r[i] * it->times[i] + 2 * it->v_k[i] + 2 * it->v_p[i]) *
				(2 * it->a_r[i] * it->times[i] + 2 * it->v_k[i] + 2 * it->v_p[i]) +
				8 * (- it->v_p[i] * it->v_p[i] - it->v_k[i] * it->v_k[i] -
				2 * it->a_r[i] * it->s[i]);

		//printf("delta: %f\n", delta);

		//printf("t: %f\t a_r: %f\t v_p: %f\n",it->times[i], it->a_r[i],it->v_p[i]);

		if (!eq(delta,0.0) && !eq(delta,-0.0)) {
			it->v_r[i] = (-(2 * it->a_r[i] * it->times[i] + 2 * it->v_k[i] +
				                       2 * it->v_p[i]) + sqrt(delta)) / (-4);
		} else {
			it->v_r[i] = (-(2 * it->a_r[i] * it->times[i] + 2 * it->v_k[i] +
							                       2 * it->v_p[i])) / (-4);
		}

		//printf("v_r: %f\n", it->v_r[i]);
		t1 = fabs(it->v_p[i] - it->v_r[i]) / it->a_r[i];
		t2 = it->times[i] - t1 - (fabs(it->v_k[i] - it->v_r[i]) / it->a_r[i]);

		//printf("t2: %f\t t1: %f\n", t2, t1);

		//it->s_acc[i] = t1 * it->v_p[i] + 0.5 * it->a_r[i] * t1 * t1;
		//it->s_uni[i] = it->v_r[i] * t2;
		calculate_s_acc_s_dec(it,i);
		calculate_s_uni(it, i);
	}

	return true;
}

bool bang_bang_profile::reduction_model_2(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {
	double a;

	a = (0.5 * (it->v_k[i] - it->v_p[i]) * (it->v_k[i] - it->v_p[i])
				+ it->v_p[i] * (it->v_k[i] - it->v_p[i])
				- it->v_k[i] * (it->v_k[i] - it->v_p[i]) )
				/ (it->s[i] - it->v_k[i] * it->times[i]);

	if ((it->v_k[i] - it->v_p[i]) / a > it->times[i]) { //drugi stopien redukcji
		if (it->s[i] == it->v_p[i] * it->times[i]) { //zabezpieczenie przed dzieleniem przez 0
			a = -1; //powoduje przejscie do nastepnego stopnia redukcji (moglaby byc dowolna ujemna liczba)
		} else {
			a = (0.5 * (it->v_k[i] - it->v_p[i]) * (it->v_k[i] - it->v_p[i])) /
				(it->s[i] - it->v_p[i]*it->times[i]);
		}

		if (a > it->a_r[i] || a <= 0) {//trzeci stopien redukcji

			double t1; //czas konca opoznienia
			double s1; // droga w etapie w ktorym redukujemy czas (przed etapem "odcinanym")
			double t2; //czas redukowanego kawalka

			t2 = it->times[i] - ((it->v_k[i] - it->v_p[i]) / it->a_r[i]);

			s1 = it->s[i] - (it->v_p[i] * (it->times[i] - t2)
			   + 0.5 * it->a_r[i] * (it->times[i] - t2) * (it->times[i] - t2));

			if (it->a_r[i] * it->a_r[i] * t2 * t2 //ujemna liczba pod pierwiastkiem, zabezpieczenie
					- 4 * it->a_r[i] * (it->v_p[i] * t2 - s1) < 0) {
				return vp_reduction(it, i);
			}

			t1 = (it->a_r[i] * t2
					- (sqrt(it->a_r[i] * it->a_r[i] * t2 * t2
					- 4 * it->a_r[i] * (it->v_p[i] * t2 - s1))) )
					/ (2 * it->a_r[i]);

			if (it->v_p[i] - it->a_r[i] * t1 < 0) {//ujemna predkosc ruchu, zabezpieczenie
				return vp_reduction(it, i);
			}

			it->v_r[i] = it->v_p[i] - it->a_r[i] * t1;
			//it->s_acc[i] = 0.5 * it->a_r[i] * t1 * t1 + it->v_r[i] * t1;
			//it->s_uni[i] = it->v_r[i] * (t2 - 2 * t1);
			calculate_s_acc_s_dec(it,i);
			calculate_s_uni(it, i);

			//printf("Reduction model 2, axis %d \t ")

			return true;
		}

		it->a_r[i] = a;
		it->v_r[i] = it->v_p[i];
		//it->s_acc[i] = 0;
		//it->s_uni[i] = it->v_r[i] * (it->times[i] - (it->v_k[i] - it->v_p[i])/it->a_r[i]);
		calculate_s_acc_s_dec(it,i);
		calculate_s_uni(it, i);
		return true;
	}

	it->a_r[i] = a;
	//printf("zredukowane a_r: %f\n", it->a_r[i]);
	it->v_r[i] = it->v_k[i];
	//it->s_acc[i] = it->v_p[i] * (it->v_k[i] - it->v_p[i])/it->a_r[i]
	 //                                 + 0.5 * (it->v_k[i] - it->v_p[i])
	//                                  * (it->v_k[i] - it->v_p[i]) / it->a_r[i];
	//it->s_uni[i] = it->v_r[i] * (it->times[i] - (it->v_k[i] - it->v_p[i]) / it->a_r[i]);
	calculate_s_acc_s_dec(it,i);
	calculate_s_uni(it, i);
	return true;
}

bool bang_bang_profile::reduction_model_3(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {
	double t1; //czas konca opoznienia

	if(it->a_r[i] * it->a_r[i] * it->times[i] * it->times[i]//liczba pierwiastkowana mniejsza od 0, zabezpieczenie
			- 4 * it->a_r[i] * (it->v_p[i] * it->times[i] - it->s[i]) < 0) {
		return vp_reduction(it, i);
	}

	t1 = (it->a_r[i] * it->times[i]
			- (sqrt(it->a_r[i] * it->a_r[i] * it->times[i] * it->times[i]
			- 4 * it->a_r[i] * (it->v_p[i] * it->times[i] - it->s[i]))) )
			/ (2 * it->a_r[i]);

	if (it->v_p[i] - it->a_r[i] * t1 < 0) {//ujemna predkosc ruchu, zabezpieczenie
		return vp_reduction(it, i);
	}

	it->v_r[i] = it->v_p[i] - it->a_r[i] * t1;
	//it->s_acc[i] = 0.5 * it->a_r[i] * t1 * t1 + it->v_r[i] * t1;
	//it->s_uni[i] = it->v_r[i] * (it->times[i] - 2 * t1);
	calculate_s_acc_s_dec(it,i);
	calculate_s_uni(it, i);
	return true;
}

bool bang_bang_profile::reduction_model_4(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {
	double a;

	//printf("############## reduction model 4 axis %d ############\n", i);

	//printf("v_p: %f\t v_k: %f\t s: %f\t times: %f\n", it->v_p[i], it->v_k[i], it->s[i], it->times[i]);

	a = (it->v_p[i] - it->v_k[i]) * (it->v_p[i] - it->v_k[i]) /
		((-2) * (it->s[i] - it->v_p[i] * it->times[i]));

	if ((it->v_p[i] - it->v_k[i]) / a > it->times[i]) { //drugi stopien redukcji
		if (it->s[i] == it->v_k[i] * it->times[i]) { //zabezpieczenie przed dzieleniem przez 0
			a = -1; //powoduje przejscie do nastepnego stopnia redukcji (moglaby byc dowolna ujemna liczba)
		} else {
			a = (0.5 * (it->v_p[i] - it->v_k[i]) * (it->v_p[i] - it->v_k[i])) /
				(it->s[i] - it->v_k[i]*it->times[i]);

		}

		if (a > it->a_r[i] || a <= 0) {//trzeci stopien redukcji
			double t1; //czas konca opoznienia (relatywny - liczac od poczatku redukowanego odcinka)
			double s1; // droga w etapie w ktorym redukujemy czas (po etapie odcinanym)
			double t2; //czas redukowanego kawalka (relatywny)

			t2 = it->times[i] - ((it->v_p[i] - it->v_k[i]) / it->a_r[i]);

			s1 = it->s[i] - (it->v_k[i] * (it->times[i] - t2)
			   + 0.5 * it->a_r[i] *(it->times[i] - t2) * (it->times[i] - t2));

			if (it->a_r[i] * it->a_r[i] * t2 * t2 //liczba pod pierwiastkiem mniejsza od 0, zabezpieczenie
				- 4 * it->a_r[i] * (it->v_k[i] * t2 - s1) < 0) {
				return vp_reduction(it, i);
			}

			t1 = (it->a_r[i] * t2
					- (sqrt(it->a_r[i] * it->a_r[i] * t2 * t2
					- 4 * it->a_r[i] * (it->v_k[i] * t2 - s1))) )
					/ (2 * it->a_r[i]);

			if ((it->v_k[i] - it->a_r[i] * t1) < 0) {//ujemna predkosc ruchu, zabezpieczenie
				return vp_reduction(it, i);
			}

			it->v_r[i] = it->v_k[i] - it->a_r[i] * t1;
			//it->s_acc[i] = 0.5 * it->a_r[i] * t1 * t1 + it->v_r[i] * t1
											//+ it->v_k[i] * (it->times[i] - t2)
											//+ 0.5 * it->a_r[i] * (it->times[i] - t2) * (it->times[i] - t2);
			//it->s_uni[i] = it->v_r[i] * (t2 - 2 * t1);
			calculate_s_acc_s_dec(it,i);
			calculate_s_uni(it, i);
			return true;
		}

		it->a_r[i] = a;
		//printf("a_r: %f", it->v_p[i]);
		it->v_r[i] = it->v_k[i];
		//it->s_acc[i] = it->v_r[i] * (it->v_p[i] - it->v_k[i])/it->a_r[i]
		 //                               + 0.5 * (it->v_p[i] - it->v_k[i]) * (it->v_p[i] - it->v_k[i])
		  //                              /it->a_r[i];
		//it->s_uni[i] = it->v_r[i] * (it->times[i]
		//							  - (it->v_p[i] - it->v_k[i])/it->a_r[i]);

		calculate_s_acc_s_dec(it,i);
		calculate_s_uni(it, i);
		return true;
	}

	it->a_r[i] = a;
	it->v_r[i] = it->v_p[i];
	//it->s_acc[i] = 0;
	//it->s_uni[i] = it->v_p[i] * (it->times[i] - (it->v_p[i] - it->v_k[i]) / it->a_r[i]);
	calculate_s_acc_s_dec(it,i);
	calculate_s_uni(it, i);
	return true;
}

bool bang_bang_profile::vp_reduction(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {

	it->v_r[i] = it->s[i]/it->times[i];
	it->v[i] = it->v_r[i]/it->v_max[i];

	//printf("------------ recursion axis: %d, v_r: %f\n", i, it->v_r[i]);
	//flushall();

	return false;
}

bool bang_bang_profile::vk_reduction(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {
	//printf("v_k redukcja w osi: %d\n", i);
	double a;
	double v_k;

	a = (2 * (it->s[i] - (it->v_p[i] * it->times[i]))) / (it->times[i] * it->times[i]);

	if (a < 0) {
		//printf("v_k stopien 2\n");
		a = (-2 * it->s[i] + 2 * it->times[i] * it->v_p[i]) / (it->times[i] * it->times[i]);
		v_k = (-1) * a * it->times[i] + it->v_p[i];
		if (eq(v_k, -0.0)) {//glupie... ale inaczej nie dziala gdy v_k jest rowne 0
			v_k = 0.0;
		}

		if (a > it->a_r[i] || v_k < 0 || v_k > it->v_k[i]) {
			//printf("v_k: %f\n", v_k);
			return vp_reduction(it, i);
		}

		it->v_k[i] = v_k;
		it->a_r[i] = a;
		it->v_r[i] = it->v_p[i];
		//it->s_acc[i] = 0;
		//it->s_uni[i] = 0;
		calculate_s_acc_s_dec(it,i);
		calculate_s_uni(it, i);
		return true;
	}

	it->v_k[i] = a * it->times[i] + it->v_p[i];
	it->a_r[i] = a;
	it->v_r[i] = it->v_k[i];
	//it->s_acc[i] = it->s[i];
	//it->s_uni[i] = 0;
	calculate_s_acc_s_dec(it,i);
	calculate_s_uni(it, i);
	return true;
}

bool bang_bang_profile::optimize_time1(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {
	double v_r;
	//double t;

	v_r = sqrt(it->a_r[i] * it->s[i] + (it->v_p[i] * it->v_p[i])/2 + (it->v_k[i] * it->v_k[i])/2);
	//it->t = t;

	if (it->v_p[i] >= it->v_k[i] && v_r < it->v_p[i]) {
		it->times[i] = (it->v_p[i] - it->v_k[i])/it->a_r[i];
		return vp_reduction(it, i);
	} else if (it->v_p[i] < it->v_k[i] && v_r < it->v_k[i]) {
		it->times[i] = (it->v_k[i] - it->v_p[i])/it->a_r[i];
		it->v_r[i] = it->v_k[i];
		return vk_reduction(it, i);
	} else {
		it->times[i] = ((v_r - it->v_p[i])/it->a_r[i] + (v_r - it->v_k[i])/it->a_r[i]);
		it->v_r[i] = v_r;
	}
	//it->t = t;
	return true;
}

bool bang_bang_profile::optimize_time2(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {

	it->v_k[i] = sqrt(2 * it->a_r[i] * it->s[i] + it->v_p[i] * it->v_p[i]);
	it->times[i] = (it->v_k[i] - it->v_p[i])/it->a_r[i];

	//printf("Time optimization model 2, axis: %d \t v_k: %f\t times: %f\n", i, it->v_k[i], it->times[i]);

	return true;
}

bool bang_bang_profile::optimize_time4(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i) {



	it->v_p[i] = sqrt(2 * it->a_r[i] * it->s[i] + it->v_k[i] * it->v_k[i]) - 0.00001;
	it->times[i] = (it->v_p[i] - it->v_k[i])/it->a_r[i];

	it->v_r[i] = it->v_p[i];//preparation for recalculation
	it->v[i] = it->v_r[i]/it->v_max[i];

	//printf("------------ recursion 2 axis: %d, v_r: %f\n", i, it->v_r[i]);
	//flushall();

	return false;
	//return vp_reduction(it, i);
}

bool bang_bang_profile::calculate_time(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i) {

	double t_acc = fabs(it->v_r[i] - it->v_p[i]) / it->a_r[i];
	double t_dec = fabs(it->v_r[i] - it->v_k[i]) / it->a_r[i];
	double t_uni = it->s_uni[i] / it->v_r[i];
	it->times[i] = t_acc + t_dec + t_uni;
	return true;
}

bool bang_bang_profile::set_v_k(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & end_it, int i) {

	if (eq(it->s[i],0)) {
		it->v_k[i] = 0;
		//printf("v_k: %f\t", it->v_k[i]);
		return true;
	}

	double temp_k = it->k[i];
	it++;

	if (it == end_it) {
		it--;
		it->v_k[i] = 0;
	} else {
		if (temp_k == it->k[i]) {
			double temp_v_k = it->v_r[i];
			//printf("v_r of next pose: %f\t", it->v_r[i]);
			it--;
			if (temp_v_k > it->v_r[i]) {
				it->v_k[i] = it->v_r[i];
				//printf("temp_v_k: %f\t", temp_v_k);
			} else {
				it->v_k[i] = temp_v_k;
				//printf("else temp_v_k: %f\t", temp_v_k);
			}
		} else {
			it--;
			it->v_k[i] = 0;
		}
	}

	//printf("v_k: %f\n", it->v_k[i]);
	return true;
}

bool bang_bang_profile::set_v_k_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & end_it) {
	bool trueFlag = true;

	for (int i = 0; i < it->axes_num; i++) {
		if (set_v_k(it, end_it, i) == false) {
			trueFlag = false;
		}
	}

	//printf("\n");
	return trueFlag;
}

bool bang_bang_profile::set_v_p_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & beginning_it) {
	bool trueFlag = true;

	for (int i = 0; i < it->axes_num; i++) {
		if (set_v_p(it, beginning_it, i) == false) {
			trueFlag = false;
		}
	}
	//printf("\n");

	return trueFlag;
}

bool bang_bang_profile::set_v_p(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & beginning_it, int i) {

	if (eq(it->s[i],0)) {
		it->v_p[i] = 0;
		//printf("v_p: %f\t", it->v_p[i]);
		return true;
	}

	if (it == beginning_it) {
		it->v_p[i] = 0;
	} else {
		it--;
		double temp_v_p = it->v_k[i];
		it++;
		it->v_p[i] = temp_v_p;
	}
	//printf("v_p: %f\t", it->v_p[i]);
	return true;
}

bool bang_bang_profile::set_model(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i) {

	if (eq(it->s[i],0)) {
		it->model[i] = 0;
		return true;
	}

	//printf("v_p: %f\t v_r: %f\t v_k: %f\n", it->v_p[i], it->v_r[i], it->v_k[i]);

	if (it->v_p[i] < it->v_r[i] && it->v_k[i] < it->v_r[i]) {
		it->model[i] = 1;
	} else if ((it->v_p[i] < it->v_r[i]
	           && eq(it->v_k[i], it->v_r[i])) || (it->v_k[i] > it->v_r[i] && it->v_k[i] > it->v_p[i])) {//tutaj bylo tez ze vk > vr (w or razem z drugim warunkiem)
		it->model[i] = 2;
	} else if (eq(it->v_p[i], it->v_r[i])
			  && eq(it->v_k[i], it->v_r[i])) { //tutaj bylo tez ze vk > vr (w or razem z drugim warunkiem)
		it->model[i] = 3;
	} else if ((eq(it->v_p[i], it->v_r[i]) || it->v_p[i] > it->v_r[i])
			   && (it->v_k[i] < it->v_r[i] || it->v_k[i] > it->v_r[i])) {
		it-> model[i] = 4;
	} else {
		printf("###################### undetermined model #######################\n");
                printf("v_p: %f\t v_r: %f\t v_k: %f\n", it->v_p[i], it->v_r[i], it->v_k[i]);
		fflush(stdout);
		it->model[i] = -1;
		return false;
	}
	return true;
}

bool bang_bang_profile::set_model_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it) {
	bool trueFlag = true;

	for (int i = 0; i < it->axes_num; i++) {
		if (set_model(it, i) == false) {
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool bang_bang_profile::calculate_s_acc_s_dec(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i) {//TODO check
	if (eq(it->s[i],0)) {
		it->s_acc[i] = 0;
		it->s_dec[i] = 0;
		return true;
	}

	if (it->v_p[i] < it->v_r[i] || eq(it->v_p[i], it->v_r[i])) {
		//printf("s_acc 1\n");
		it->s_acc[i] = it->v_p[i] * fabs(it->v_r[i] - it->v_p[i]) / it->a_r[i] //distance covered by the uniform motion with initial velocity (s = v * t)
		        + (0.5 * it->a_r[i] * (fabs(it->v_r[i]         // + distance covered by acceleration neglecting the initial velocity (s = 1/2 * a * t^2)
				- it->v_p[i]) / it->a_r[i]) * (fabs(it->v_r[i]
				- it->v_p[i]) / it->a_r[i]));

	} else {
		//printf("s_acc 2\n");
		it->s_acc[i] = it->v_p[i] * fabs(it->v_p[i] - it->v_r[i]) / it->a_r[i]
		        - (0.5 * it->a_r[i] * (fabs(it->v_p[i]
				- it->v_r[i]) / it->a_r[i]) * (fabs(it->v_p[i] - it->v_r[i])
				/ it->a_r[i]));
	}

	if (it->v_k[i] < it->v_r[i] || eq(it->v_k[i], it->v_r[i])) {
		//printf("s_dec1\n");
		it->s_dec[i] = it->v_r[i] * fabs(it->v_r[i] - it->v_k[i])
				/ it->a_r[i] - (0.5 * it->a_r[i] * (fabs(it->v_r[i]
				- it->v_k[i]) / it->a_r[i]) * (fabs(it->v_r[i] - it->v_k[i])
				/ it->a_r[i]));
		//printf("s_dec: %f\n", it->s_dec[i]);
	} else {
		//printf("s_dec 2\n");
		it->s_dec[i] = it->v_r[i] * fabs(it->v_k[i] - it->v_r[i])
				/ it->a_r[i] + (0.5 * it->a_r[i] * (fabs(it->v_k[i]
				- it->v_r[i]) / it->a_r[i]) * (fabs(it->v_k[i]
				- it->v_r[i]) / it->a_r[i]));
		//printf("s_dec: %f\n", it->s_dec[i]);
	}

	return true;
}

bool bang_bang_profile::calculate_s_acc_s_dec_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it) {

	bool trueFlag = true;

	for (int i = 0; i < it->axes_num; i++) {
		if (calculate_s_acc_s_dec(it, i) == false) {
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool bang_bang_profile::check_s_acc_s_decc(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i) {

	if (it->s_acc[i] + it->s_dec[i] > it->s[i]) {
		return false;
	} else {
		return true;
	}
}

bool bang_bang_profile::calculate_s_uni_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it) {
	bool trueFlag = true;

	for (int i = 0; i < it->axes_num; i++) {
		if (calculate_s_uni(it, i) == false) {
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool bang_bang_profile::calculate_s_uni(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i) {

	if (eq(it->s[i],0)) {
		it->s_uni[i] = 0;
		return true;
	}

	it->s_uni[i] = it->s[i] - (it->s_acc[i] + it->s_dec[i]);

	return true;
}

bool bang_bang_profile::calculate_acc_uni_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, const double & mc) {
	bool trueFlag = true;

	for (int i = 0; i < it->axes_num; i++) {
		if (calculate_acc_uni(it, mc, i) == false) {
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool bang_bang_profile::calculate_acc_uni(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, const double & mc, int i) {

	if (eq(it->s[i],0)) {
		it->uni[i] = 0;
		it->acc[i] = 0;
		return true;
	}

	it->acc[i] = fabs((it->v_r[i] - it->v_p[i])
			/ (it->a_r[i] * mc));
	it->uni[i] = (it->times[i] - (fabs(it->v_r[i] - it->v_k[i])
			/ it->a_r[i])) / mc - it->acc[i];
	return true;
}

bool bang_bang_profile::reduction_axis(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i) {

	if (it->model[i] == 1) {
		if (!reduction_model_1(it, i)) {
			return false;
		}
	} else if (it->model[i] == 2) {
		if (!reduction_model_2(it, i)) {
			return false;
		}
	} else if (it->model[i] == 3) {
		if (!reduction_model_3(it, i)) {
			return false;
		}
	} else if (it->model[i] == 4) {
		if (!reduction_model_4(it, i)) {
			return false;
		}
	} else {
		//TODO ??
	}

	return true;
}

bool bang_bang_profile::optimize_time_axis(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i) {

	if (it->model[i] == 1) {
			if (!optimize_time1(it, i)) {
				return false;
			}
		} else if (it->model[i] == 2) {
			if (!optimize_time2(it, i)) {
				return false;
			}
		} else if (it->model[i] == 3) {
			return true;
		} else if (it->model[i] == 4) {
			if (!optimize_time4(it, i)) {
				return false;
			}
		} else {
			//TODO ??
		}
	return true;
}

bool bang_bang_profile::calculate_v_r_a_r_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it) {
	bool trueFlag = true;

	for (int i = 0; i < it->axes_num; i++) {
		if (calculate_v_r_a_r(it, i) == false) {
			trueFlag = false;
		}
        }

	return trueFlag;
}

bool bang_bang_profile::calculate_v_r_a_r(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i) {

	it->v_r[i] = it->v[i] * it->v_max[i];
	it->a_r[i] = it->a[i] * it->a_max[i];

	return true;
}

void bang_bang_profile::clean_up_pose(std::vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it) {

	for (int i = 0; i < it->axes_num; i++) {
		it->a_r[i] = 0;
		it->v_r[i] = 0;
		it->model[i] = 0;
		it->s_acc[i] = 0;
		it->s_dec[i] = 0;
		it->acc[i] = 0;
		it->s_uni[i] = 0;
		it->v_k[i] = 0;
		it->v_p[i] = 0;
		it->uni[i] = 0;
		it->k[i] = 0;
                it->times[i] = 0;
                it->s[i] = 0;
	}
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
