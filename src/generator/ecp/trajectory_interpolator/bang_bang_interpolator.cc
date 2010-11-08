/**
 * @file
 * @brief Contains definitions of the methods of bang_bang_interpolator class.
 * @author rtulwin
 * @ingroup generators
 */

#include "bang_bang_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

using namespace std;

bang_bang_interpolator::bang_bang_interpolator() {
	// TODO Auto-generated constructor stub

}

bang_bang_interpolator::~bang_bang_interpolator() {
	// TODO Auto-generated destructor stub
}

bool bang_bang_interpolator::interpolate_relative_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {

	vector<double> coordinates (it->axes_num);
	for (int i = 0; i < it->interpolation_node_no; i++) {
		for (int j = 0; j < it->axes_num; j++) {
			if (fabs(it->s[j]) < 0.0000001) {
				coordinates[j] = 0;
			} else {
				coordinates[j] = generate_next_coordinate(i+1, it->interpolation_node_no, it->start_position[j], it->v_p[j], it->v_r[j], it->v_k[j], it->a_r[j], it->k[j], it->acc[j], it->uni[j], it->s_acc[j], it->s_uni[j], lib::RELATIVE, mc);
			}
			//TODO add checking the correctness of the returned values
		}
		cv.push_back(coordinates);
	}

	return true;
}

bool bang_bang_interpolator::interpolate_absolute_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc)
{
	vector <double> coordinates(it->axes_num);
	for (int i = 0; i < it->interpolation_node_no; i++) {
		for (int j = 0; j < it->axes_num; j++) {
			coordinates[j] = generate_next_coordinate(i + 1, it->interpolation_node_no, it->start_position[j], it->v_p[j], it->v_r[j], it->v_k[j], it->a_r[j], it->k[j], it->acc[j], it->uni[j], it->s_acc[j], it->s_uni[j], lib::ABSOLUTE, mc);
		}
		//TODO add checking the correctness of the returned values

		cv.push_back(coordinates);
	}

	return true;
}

//TODO divide into smaller functions and comment code
double bang_bang_interpolator::generate_next_coordinate(int node_counter, int interpolation_node_no, double start_position, double v_p, double v_r, double v_k, double a_r, double k, double acc, double uni, double s_acc, double s_uni, lib::MOTION_TYPE motion_type, const double mc) {
	//funkcja obliczajaca polozenie w danym makrokroku

	double next_position = 0;

	if (node_counter < acc + 1) { //pierwszy etap
		if (v_p <= v_r) { //przyspieszanie w pierwszym etapie

			if (motion_type == lib::ABSOLUTE) {//tryb absolute

				if (acc > (node_counter - 1) && acc < (node_counter)) {//przyspieszanie wchodzi na jednostajny
					if (acc + uni < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku

						next_position = start_position + k * (acc * v_p * mc + acc * acc * a_r * mc * mc / 2
								+ uni * mc * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k * (v_r * (node_counter - acc - uni) * mc - mc * mc * (node_counter
									- uni - acc) * (node_counter - uni - acc) * a_r / 2);
						} else {//przyspiesznie w trzecim etapie
							next_position += k * (v_r * (node_counter - acc - uni) * mc + mc * mc * (node_counter
									- uni - acc) * (node_counter - uni - acc) * a_r / 2);
						}
					} else {//przyspieszenie + poczatek jednostajnego
						next_position = start_position + k * (v_p * acc * mc + acc * acc * mc * mc * a_r
								/ 2 + v_r * (node_counter - acc) * mc);
					}
				} else {//normalne przyspieszanie
					next_position = start_position + k * (node_counter * v_p * mc + node_counter * node_counter * a_r
							* mc * mc / 2);
				}

			} else if (motion_type == lib::RELATIVE) {//tryb relatywny
				if (acc > (node_counter - 1) && acc < (node_counter)) {//przyspieszanie wchodzi na jednostajny
					if (acc + uni < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku

						next_position = k * ((1 - node_counter + acc) * v_r * mc - (1 - node_counter + acc) * (1
								- node_counter + acc) * a_r * mc * mc / 2 + uni * mc * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k * (v_r * (node_counter - acc - uni) * mc - mc * mc * (node_counter
									- uni - acc) * (node_counter - uni - acc) * a_r / 2);
						} else {//przyspiesznie w trzecim etapie
							next_position += k * (v_r * (node_counter - acc - uni) * mc + mc * mc * (node_counter
									- uni - acc) * (node_counter - uni - acc) * a_r / 2);
						}
					} else {//przyspieszenie + poczatek jednostajnego
						next_position = k * (v_r * (1 - node_counter + acc) * mc - (1 - node_counter + acc) * (1
								- node_counter + acc) * mc * mc * a_r / 2 + v_r * (node_counter - acc) * mc);
					}
				} else {//normalne przyspieszanie
					next_position = k * (v_p * mc + (node_counter - 1) * a_r * mc * mc + (a_r * mc * mc) / 2);
				}
			}

		} else { //hamowanie w pierwszym etapie
			if (motion_type == lib::ABSOLUTE) {
				if ((acc) > (node_counter-1) && (acc) < (node_counter)) {

					if (acc + uni < node_counter) {

						next_position = start_position + k * (acc * v_p * mc - acc * acc * a_r * mc * mc / 2
								+ uni * mc * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k * (v_r * (node_counter - acc - uni) * mc - mc * mc * (node_counter
									- uni - acc) * (node_counter - uni - acc) * a_r / 2);
						} else {//przyspieszanie w trzecim etapie
							next_position += k * (v_r * (node_counter - acc - uni) * mc + mc * mc * (node_counter
									- uni - acc) * (node_counter - uni - acc) * a_r / 2);
						}
					} else {//hamowanie + poczatek jednostajnego
						next_position = start_position + k * (v_p * acc * mc - acc * acc * mc * mc * a_r
								/ 2 + v_r * (node_counter - acc) * mc);
					}
				} else {//normalne hamowanie
					next_position = start_position + k * (node_counter * mc * v_p - node_counter * node_counter * mc
							* mc * a_r / 2);
				}

			} else if (motion_type == lib::RELATIVE) {
				if (acc > (node_counter - 1) && acc < (node_counter)) {//hamowanie wchodzi na jednostajny
					if (acc + uni < node_counter) {//specjalny przypadek gdy faza jednostajnego zamyka sie w jednym makrokroku
						next_position = k * ((1 - node_counter + acc) * v_r * mc + (1 - node_counter + acc) * (1
								- node_counter + acc) * a_r * mc * mc / 2 + uni * mc * v_r);

						if (v_r > v_k) {//hamowanie w trzecim etapie
							next_position += k * (v_r * (node_counter - acc - uni) * mc - mc * mc * (node_counter
									- uni - acc) * (node_counter - uni - acc) * a_r / 2);
						} else {//przyspieszanie w trzecim etapie
							next_position += k * (v_r * (node_counter - acc - uni) * mc + mc * mc * (node_counter
									- uni - acc) * (node_counter - uni - acc) * a_r / 2);
						}
					} else {//hamowanie + poczatek jednostajnego
						next_position = k * (v_r * (1 - node_counter + acc) * mc + (1 - node_counter + acc) * (1
								- node_counter + acc) * mc * mc * a_r / 2 + v_r * (node_counter - acc) * mc);
					}
				} else {//normalne hamowanie
					next_position = k * (v_p * mc - ((node_counter - 1) * a_r * mc * mc + (a_r * mc * mc) / 2));
				}
			}
		}
	} else if (node_counter <= acc + uni + 1) { // drugi etap - ruch jednostajny
		if (motion_type == lib::ABSOLUTE) {
			if ((acc + uni) > (node_counter - 1) && (acc + uni) < (node_counter)) {//jednostajny wchodzi w faze trzecia

				next_position = start_position + k * (s_acc + s_uni);

				if (v_r > v_k) {//hamowanie w 3 etapie
					next_position += k * (v_r * (node_counter - uni - acc) * mc - (node_counter - uni - acc)
							* (node_counter - uni - acc) * mc * mc * a_r / 2);
				} else {//przyspieszanie w 3 etapie
					next_position += k * (v_r * (node_counter - uni - acc) * mc + (node_counter - uni - acc)
							* (node_counter - uni - acc) * mc * mc * a_r / 2);
				}
			} else { //normalny jednostajny
				next_position = start_position + k * (s_acc + ((node_counter - acc) * mc) * v_r);
			}
		} else if (motion_type == lib::RELATIVE) {
			if ((acc + uni) > (node_counter - 1) && (acc + uni) < (node_counter)) {//jednostajny wchodzi w faze trzecia
				next_position = k* v_r * mc * (1 - node_counter + acc + uni);

				if (v_r > v_k) {//hamowanie w 3 etapie
					next_position += k * (v_r * (node_counter - uni - acc) * mc - (node_counter - uni - acc)
							* (node_counter - uni - acc) * mc * mc * a_r / 2);
				} else {//przyspieszanie w 3 etapie
					next_position += k * (v_r * (node_counter - uni - acc) * mc + (node_counter - uni - acc)
							* (node_counter - uni - acc) * mc * mc * a_r / 2);
				}
			} else { //normalny jednostajny
				next_position = k * v_r * mc;
			}
		}

	} else if (node_counter <= interpolation_node_no) { //trzeci etap

		if (v_k <= v_r) { //hamowanie w trzecim etapie
			if (motion_type == lib::ABSOLUTE) {
				next_position = start_position + k * (s_acc + s_uni + ((node_counter - uni - acc) * mc) * v_r
						- ((node_counter - uni - acc) * mc) * ((node_counter - uni - acc) * mc) * a_r / 2);
			} else if (motion_type == lib::RELATIVE) {
				next_position = k * ((v_k + (interpolation_node_no - node_counter) * a_r * mc) * mc + (a_r * mc * mc)
						/ 2);
			}
		} else { //przyspieszanie w trzecim etapie
			if (motion_type == lib::ABSOLUTE) {
				next_position = start_position + k * (s_acc + s_uni + ((node_counter - uni - acc) * mc) * v_r
						+ a_r * ((node_counter - uni - acc) * mc) * ((node_counter - uni - acc) * mc) / 2);
			} else if (motion_type == lib::RELATIVE) {
				next_position = k * ((v_k - (interpolation_node_no - node_counter) * a_r * mc) * mc - (a_r * mc * mc)
						/ 2);
			}
		}
	}

	return next_position;
}

double bang_bang_interpolator::generate_relative_coordinate(int node_counter, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int axis_num, const double mc) {
	return generate_next_coordinate(node_counter+1, it->interpolation_node_no, it->start_position[axis_num], it->v_p[axis_num], it->v_r[axis_num], it->v_k[axis_num], it->a_r[axis_num], it->k[axis_num], it->acc[axis_num], it->uni[axis_num], it->s_acc[axis_num], it->s_uni[axis_num], lib::RELATIVE, mc);
}

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
