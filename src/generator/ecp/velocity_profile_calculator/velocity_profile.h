/**
 * @file
 * @brief Contains declarations and definitions of the methods of velocity_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _VELOCITY_PROFILE_H_
#define _VELOCITY_PROFILE_H_

#include <cstdio>

#include <algorithm>
#include <vector>

#include <base/lib/trajectory_pose/trajectory_pose.h>
#include <base/lib/mrmath/mrmath.h>

namespace mrrocpp {
    namespace ecp {
        namespace common {
            namespace generator {
                namespace velocity_profile_calculator {

                    /**
                     * @brief Base class for all of the velocity profile calculators.
                     *
                     * Usually any velocity profile calculator contains methods used to create the description
                     * of the velocity profile f.g. bang bang velocity profile. This information is usually stored in the appropriate trajectory_pose class.
                     */
                    template <class Pos>
                    class velocity_profile {
                    public:

                        /**
                         * Constructor.
                         */
                        velocity_profile() {

                        }

                        /**
                         * Destructor.
                         */
                        virtual ~velocity_profile() {

                        }

                        /**
                         * Method comparing two double values.
                         * @return true if values are the same
                         */
                        bool eq(double a, double b) {
                            const double epsilon = 0.0000001;
                            const double& diff = a - b;
                            return diff < epsilon && diff > -epsilon;
                        }

                        /**
                         * Checks if the distance covered in the given axis is equal to 0.
                         * @return true if the distance covered in the given pose and axis is equal to 0.
                         */
                        bool check_if_no_movement(typename std::vector<Pos>::iterator & it, int i) {
                            if (eq(it->s[i], 0.0)) {
                                return true;
                            } else {
                                return false;
                            }
                        }

                        /**
                         * Calculates http://allegro.pl/gniazdo-jack-6-3mm-metal-mono-jck229-neutrik-i1850074806.htmldistance for all of the axes in a single trajectory pose and sets the directions of movements of absolute type.
                         * @param it iterator to the list of positions
                         * @return true if the set of the distance and direction was successful (usually is if the vectors start_position and coordinates were initiated and filled in before)
                         */
                        bool calculate_absolute_distance_direction_pose(typename std::vector<Pos>::iterator & it) {

                            if (it->coordinates.size() < it->axes_num || it->start_position.size() < it->axes_num) {
                                return false;
                            }

                            it->s.clear();
                            it->k.clear();
                            for (std::size_t i = 0; i < it->axes_num; i++) {
                                it->s.push_back(fabs(it->coordinates[i] - it->start_position[i]));
                                if (eq(it->coordinates[i] - it->start_position[i], 0)) {
                                    it->k.push_back(0);
                                } else if (it->coordinates[i] - it->start_position[i] > 0) {
                                    it->k.push_back(1);
                                } else {
                                    it->k.push_back(-1);
                                }
                            }

                            return true;
                        }

                        /**
                         * Calculates distance for all of the axes in a single trajectory pose and sets the directions of movements of relative type.
                         * @param it iterator to the list of positions
                         * @return true if the set of the distance and direction was successful (usually is if the coordinates vector was initiated and filled in before)
                         */
                        bool calculate_relative_distance_direction_pose(typename std::vector<Pos>::iterator & it) {

                            if (it->coordinates.size() < it->axes_num) {
                                return false;
                            }

                            it->s.clear();
                            it->k.clear();
                            for (std::size_t i = 0; i < it->axes_num; i++) {
                                it->s.push_back(fabs(it->coordinates[i]));

                                if (eq(it->coordinates[i], 0)) {
                                    it->k.push_back(0);
                                } else if (it->coordinates[i] > 0) {
                                    it->k.push_back(1);
                                } else {
                                    it->k.push_back(-1);
                                }
                            }

                            return true;
                        }
                        /**
                         * Calculates time for the given velocity and distance for a single axis in a single pose.
                         * @param it iterator to the list of positions
                         * @param i number of axis for which the calculations are performed
                         * @return true if the time was calculated successfully (if all of the necessary information was provided)
                         */
                        virtual bool calculate_time(typename std::vector<Pos>::iterator & it, int i) = 0;

                        /**
                         * Calculates time for the given velocity and distance for all axes in a single pose.
                         * @param it iterator to the list of positions
                         * @return true if the time was calculated successfully (if all of the necessary information was provided)
                         */
                        bool calculate_time_pose(typename std::vector<Pos>::iterator & it) {

                            bool trueFlag = true;

                            for (std::size_t i = 0; i < it->axes_num; i++) {
                                if (calculate_time(it, i) == false) {
                                    trueFlag = false;
                                }
                            }

                            return trueFlag;
                        }

                        /**
                         * Calculates the longest time from the times vector and stores it in t variable in the pose. Extends t to make it the multiplicity of the macrostep time.
                         * @param it iterator to the list of positions
                         * @param mc macrostep time
                         * @return true if the set of the time was successful (usually is if the vector times was initiated and filled in before)
                         */
                        bool calculate_pose_time(typename std::vector<Pos>::iterator & it, const double & mc) {
                            if (it->times.size() == it->axes_num) {
                                double t_max = *max_element(it->times.begin(), it->times.end());

                                if (eq(t_max, 0.0)) {
                                    it->t = 0;
                                    return true;
                                }

                                if (ceil(t_max / mc) * mc != t_max) { //extend the pose time to be the multiplicity of the macrostep time
                                    t_max = ceil(t_max / mc);
                                    t_max = t_max * mc;
                                    it->t = t_max;
                                } else {
                                    it->t = t_max;
                                }

                                return true;
                            } else {
                                return false;
                            }
                        }

                        /**
                         * Sets all of the values in %times vector to t.
                         * @return true if the calculation was successful
                         */
                        bool set_times_to_t(typename std::vector<Pos>::iterator & it) {
                            for (std::size_t i = 0; i < it->axes_num; i++) {
                                it->times[i] = it->t;
                            }
                            return true;
                        }

                        /**
                         * Calculates the relative angle axis vector. The relative distance is calculated between the start position and coordinates vectors.
                         * Result is stored in coordinates vector of the current pose.
                         * @return true if the calculation was successful
                         */
                        bool calculate_relative_angle_axis_vector(typename std::vector<Pos>::iterator & it) {
                            lib::Homog_matrix start_position_matrix;
                            lib::Homog_matrix desired_position_matrix;
                            lib::Xyz_Angle_Axis_vector relative_angle_axis_vector;
                            lib::Xyz_Angle_Axis_vector relative_angle_axis_vector_with_changed_configuration;
                            //lib::Ft_tr xsi_star_matrix;

                            double start_position[6];
                            double coordinates[6];

                            if (it->start_position.size() != 6) {
                                return false;
                            }

                            memcpy(start_position, &it->start_position[0], sizeof (double) * it->start_position.size()); //it->start_position.size() should always be equal to 6
                            memcpy(coordinates, &it->coordinates[0], sizeof (double) * it->coordinates.size());

                            start_position_matrix.set_from_xyz_angle_axis(start_position);
                            desired_position_matrix.set_from_xyz_angle_axis(coordinates);
                            ((!start_position_matrix) * desired_position_matrix).get_xyz_angle_axis(relative_angle_axis_vector);

                            it->xsi_star_matrix = lib::Ft_tr(!start_position_matrix.return_with_with_removed_translation());
                            relative_angle_axis_vector_with_changed_configuration = it->xsi_star_matrix * relative_angle_axis_vector;

                            relative_angle_axis_vector_with_changed_configuration.to_vector(it->coordinates);

                            //printf("relative vector: \n");
                            //printf("%f\t%f\t%f\t%f\t%f\t%f\n", it->coordinates[0], it->coordinates[1], it->coordinates[2], it->coordinates[3], it->coordinates[4], it->coordinates[5]);

                            return true;
                        }

                    };

                } // namespace velocity_profile_calculator
            } // namespace generator
        } // namespace common
    } // namespace ecp
} // namespace mrrocpp

#endif /* _VELOCITY_PROFILE_H_ */
