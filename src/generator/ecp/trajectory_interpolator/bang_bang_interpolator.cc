/*
 * bang_bang_interpolator.cc
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#include "bang_bang_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

bang_bang_interpolator::bang_bang_interpolator() {
	// TODO Auto-generated constructor stub

}

bang_bang_interpolator::~bang_bang_interpolator() {
	// TODO Auto-generated destructor stub
}

bool bang_bang_interpolator::interpolate_relative_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double & mc) {

	vector<double> coordinates (it->axes_num);
	for (int i = 0; i < it->interpolation_node_no; i++) {
		for (int j = 0; j < it->axes_num; j++) {
			if (it->s[j] == 0.0) {
				coordinates[j] = 0;
			} else {
				coordinates[j] = generate_next_coords(i+1, it->interpolation_node_no, it->start_position[j], it->v_p[j], it->v_r[j], it->v_k[j], it->a_r[j], it->k[j], it->acc[j], it->uni[j], it->s_acc[j], it->s_uni[j], lib::RELATIVE);
			}
		}
		cv.push_back(coordinates);
	}

	return true;
}

bool bang_bang_interpolator::interpolate_absolute_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double & mc) {

	int i,j;

	vector<double> coordinates (it->axes_num);
	for (i = 0; i < it->interpolation_node_no; i++) {
		for (j = 0; j < it->axes_num; j++) {
			coordinates[j] = generate_next_coords(i+1, it->interpolation_node_no, it->start_position[j], it->v_p[j], it->v_r[j], it->v_k[j], it->a_r[j], it->k[j], it->acc[j], it->uni[j], it->s_acc[j], it->s_uni[j], lib::ABSOLUTE);
		}
		cv.push_back(coordinates);
	}

	return true;
}

double bang_bang_interpolator::generate_next_coords(int node_counter, int interpolation_node_no, double start_position, double v_p, double v_r, double v_k, double a_r, double k, double przysp, double jedn, double s_przysp, double s_jedn, lib::MOTION_TYPE type)
{

	//funkcja obliczajaca polozenie w danym makrokroku

	double next_position = 0;

	double tk = 10 * 0.002;

	//printf("przysp: %f\t jedn: %f\n", przysp, jedn);

	if (node_counter < przysp + 1) { //pierwszy etap
		if (v_p <= v_r) { //przyspieszanie w pierwszym etapie
			//printf("start pos: %f\t node counter: %d\n", start_position, node_counter);
			//printf(" przysp1 %d ", node_counter);
			if (type == lib::ABSOLUTE) {//tryb absolute

				if (przysp > (node_counter - 1) && przysp < (node_counter)) {//przyspieszanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku

						next_position = start_position + k * (przysp * v_p * tk + przysp * przysp * a_r * tk * tk / 2
								+ jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk - a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp - jedn) * tk - tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						} else {//przyspiesznie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk + a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp - jedn) * tk + tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						}
					} else {//przyspieszenie + poczatek jednostajnego
						//printf("przyspieszanie wchodzi na jednostajny, macrostep liczony od 1: %d\n", node_counter);
						//next_position = start_position + k * ((node_counter-1)*v_p*tk + (node_counter-1)*(node_counter-1)*a_r*tk*tk/2);
						//next_position += k * (v_p*tk + a_r*tk*tk/2 + (v_p+a_r*(node_counter-1) *tk)) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp);
						next_position = start_position + k * (v_p * przysp * tk + przysp * przysp * tk * tk * a_r
								/ 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne przyspieszanie
					next_position = start_position + k * (node_counter * v_p * tk + node_counter * node_counter * a_r
							* tk * tk / 2);
				}

			} else if (type == lib::RELATIVE) {//tryb relatywny
				if (przysp > (node_counter - 1) && przysp < (node_counter)) {//przyspieszanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku

						next_position = k * ((1 - node_counter + przysp) * v_r * tk - (1 - node_counter + przysp) * (1
								- node_counter + przysp) * a_r * tk * tk / 2 + jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk - a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp - jedn) * tk - tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						} else {//przyspiesznie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk + a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp - jedn) * tk + tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						}
					} else {//przyspieszenie + poczatek jednostajnego
						//printf("przyspieszanie wchodzi na jednostajny, macrostep liczony od 1: %d\n", node_counter);
						//next_position = start_position + k * ((node_counter-1)*v_p*tk + (node_counter-1)*(node_counter-1)*a_r*tk*tk/2);
						//next_position += k * (v_p*tk + a_r*tk*tk/2 + (v_p+a_r*(node_counter-1) *tk)) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp);
						next_position = k * (v_r * (1 - node_counter + przysp) * tk - (1 - node_counter + przysp) * (1
								- node_counter + przysp) * tk * tk * a_r / 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne przyspieszanie
					next_position = k * (v_p * tk + (node_counter - 1) * a_r * tk * tk + (a_r * tk * tk) / 2);
				}
			}

		} else { //hamowanie w pierwszym etapie
			//printf(" ham1 %d ", node_counter);
			if (type == lib::ABSOLUTE) {
				if ((przysp) > (node_counter-1) && (przysp) < (node_counter)) {

					if (przysp + jedn < node_counter) {

						next_position = start_position + k * (przysp * v_p * tk - przysp * przysp * a_r * tk * tk / 2
								+ jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k * (v_r * (node_counter - przysp - jedn) * tk - tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
							//next_position += k * (v_p*tk - a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk - a_r*tk*tk/2) * (node_counter - przysp - jedn);
						} else {//przyspieszanie w trzecim etapie
							next_position += k * (v_r * (node_counter - przysp - jedn) * tk + tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
							//next_position += k * (v_p*tk - a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk + a_r*tk*tk/2) * (node_counter - przysp - jedn);
						}
					} else {//hamowanie + poczatek jednostajnego
						//next_position = start_position + k * ((node_counter-1)*v_p*tk + (node_counter-1)*(node_counter-1)*a_r*tk*tk/2);
						//next_position += k * (v_p*tk - a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp);
						next_position = start_position + k * (v_p * przysp * tk - przysp * przysp * tk * tk * a_r
								/ 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne hamowanie
					next_position = start_position + k * (node_counter * tk * v_p - node_counter * node_counter * tk
							* tk * a_r / 2);
				}

			} else if (type == lib::RELATIVE) {
				if (przysp > (node_counter - 1) && przysp < (node_counter)) {//hamowanie wchodzi na jednostajny
					if (przysp + jedn < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku
						next_position = k * ((1 - node_counter + przysp) * v_r * tk + (1 - node_counter + przysp) * (1
								- node_counter + przysp) * a_r * tk * tk / 2 + jedn * tk * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk - a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp - jedn) * tk - tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						} else {//przyspieszanie w trzecim etapie
							//next_position += k * (v_p*tk + a_r*tk*tk/2) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp - (node_counter - przysp - jedn)) + k * (v_r*tk + a_r*tk*tk/2) * (node_counter - przysp - jedn);
							next_position += k * (v_r * (node_counter - przysp - jedn) * tk + tk * tk * (node_counter
									- jedn - przysp) * (node_counter - jedn - przysp) * a_r / 2);
						}
					} else {//hamowanie + poczatek jednostajnego
						//printf("przyspieszanie wchodzi na jednostajny, macrostep liczony od 1: %d\n", node_counter);
						//next_position = start_position + k * ((node_counter-1)*v_p*tk + (node_counter-1)*(node_counter-1)*a_r*tk*tk/2);
						//next_position += k * (v_p*tk + a_r*tk*tk/2 + (v_p+a_r*(node_counter-1) *tk)) * (przysp - (node_counter-1)) + k * tk * v_r * (node_counter - przysp);
						next_position = k * (v_r * (1 - node_counter + przysp) * tk + (1 - node_counter + przysp) * (1
								- node_counter + przysp) * tk * tk * a_r / 2 + v_r * (node_counter - przysp) * tk);
					}
				} else {//normalne hamowanie
					next_position = k * (v_p * tk - ((node_counter - 1) * a_r * tk * tk + (a_r * tk * tk) / 2));
				}
			}
		}
		//printf("%f\t", next_position);
	} else if (node_counter <= przysp + jedn + 1) { // drugi etap - ruch jednostajny
		//printf(" jedn %d ", node_counter);
		if (type == lib::ABSOLUTE) {
			if ((przysp + jedn) > (node_counter - 1) && (przysp + jedn) < (node_counter)) {//jednostajny wchodzi w faze trzecia

				next_position = start_position + k * (s_przysp + s_jedn);

				if (v_r > v_k) {//hamowanie w 3 etapie
					//printf("wchodzi macrostep: %d\n", node_counter);
					next_position += k * (v_r * (node_counter - jedn - przysp) * tk - (node_counter - jedn - przysp)
							* (node_counter - jedn - przysp) * tk * tk * a_r / 2);
				} else {//przyspieszanie w 3 etapie
					next_position += k * (v_r * (node_counter - jedn - przysp) * tk + (node_counter - jedn - przysp)
							* (node_counter - jedn - przysp) * tk * tk * a_r / 2);
				}
			} else { //normalny jednostajny
				next_position = start_position + k * (s_przysp + ((node_counter - przysp) * tk) * v_r);
			}
			//printf("next_pos: %f\n", next_position);
		} else if (type == lib::RELATIVE) {
			if ((przysp + jedn) > (node_counter - 1) && (przysp + jedn) < (node_counter)) {//jednostajny wchodzi w faze trzecia

				printf("v_r: %f\t node: %d\t przysp: %f\t jedn: %f\t k: %f\t a_r: %f\t tk: %f\n", v_r, node_counter, przysp, jedn, k, a_r, tk);

				next_position = k* v_r * tk * (1 - node_counter + przysp + jedn);

				if (v_r > v_k) {//hamowanie w 3 etapie
					//printf("wchodzi macrostep: %d\n", node_counter);
					next_position += k * (v_r * (node_counter - jedn - przysp) * tk - (node_counter - jedn - przysp)
							* (node_counter - jedn - przysp) * tk * tk * a_r / 2);
					printf("hamowanie w 3 etapie: %f \n", next_position);
				} else {//przyspieszanie w 3 etapie
					next_position += k * (v_r * (node_counter - jedn - przysp) * tk + (node_counter - jedn - przysp)
							* (node_counter - jedn - przysp) * tk * tk * a_r / 2);
				}
			} else { //normalny jednostajny
				next_position = k * v_r * tk;
			}
			//printf("%f\t", next_position);
		}

	} else if (node_counter <= interpolation_node_no) { //trzeci etap

		if (v_k <= v_r) { //hamowanie w trzecim etapie
			//printf(" ham2 %d ", node_counter);
			if (type == lib::ABSOLUTE) {
				//printf("start: %f\t node_counter: %d\t s przysp: %f\t s_jedn: %f\n", start_position, node_counter, s_przysp, s_jedn);
				//printf("next_pos: %f\n", next_position);
				next_position = start_position + k * (s_przysp + s_jedn + ((node_counter - jedn - przysp) * tk) * v_r
						- ((node_counter - jedn - przysp) * tk) * ((node_counter - jedn - przysp) * tk) * a_r / 2);
				//printf("next_pos hamowanie: %f\n", next_position);
			} else if (type == lib::RELATIVE) {
				next_position = k * ((v_k + (interpolation_node_no - node_counter) * a_r * tk) * tk + (a_r * tk * tk)
						/ 2);
			}
			//printf("next pos: %f\t node: %d\t", next_position, node_counter);
		} else { //przyspieszanie w trzecim etapie
			//printf(" przysp2 %d ", node_counter);
			if (type == lib::ABSOLUTE) {
				next_position = start_position + k * (s_przysp + s_jedn + ((node_counter - jedn - przysp) * tk) * v_r
						+ a_r * ((node_counter - jedn - przysp) * tk) * ((node_counter - jedn - przysp) * tk) / 2);
			} else if (type == lib::RELATIVE) {
				next_position = k * ((v_k - (interpolation_node_no - node_counter) * a_r * tk) * tk - (a_r * tk * tk)
						/ 2);
			}
		}
		//printf("%f\t", next_position);
	}

	//printf("next pos: %f\t node: %d\t", next_position, node_counter);
	//flushall();
	return next_position;
}

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
