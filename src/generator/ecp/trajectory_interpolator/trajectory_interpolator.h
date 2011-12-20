/**
 * @file
 * @brief Contains declarations and definitions of the methods of trajectory_interpolator class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _TRAJECTORY_INTERPOLATOR_H_
#define _TRAJECTORY_INTERPOLATOR_H_

#include <vector>
#include <cstdio>

#include "base/lib/trajectory_pose/trajectory_pose.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
    namespace ecp {
        namespace common {
            namespace generator {
                namespace trajectory_interpolator {

                    /**
                     * @brief Base class for all trajectory interpolators.
                     *
                     * @author rtulwin
                     * @ingroup generators
                     */
                    template <class Pos>
                    class trajectory_interpolator {
                    public:

                        /**
                         * Constructor.
                         */
                        trajectory_interpolator() {

                        }

                        /**
                         * Destructor.
                         */
                        virtual ~trajectory_interpolator() {

                        }
                        /**
                         * Method interpolates the relative type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
                         * @param pose_vector_iterator iterator to the list of positions
                         * @param coordinate_vector list of coordinates
                         * @param mc time of a single macrostep
                         * @return true if the interpolation was successful
                         */
                        virtual bool interpolate_relative_pose(typename std::vector<Pos>::iterator & pose_vector_iterator, std::vector<std::vector<double> > & coordinate_vector, const double mc) = 0;
                        /**
                         * Method interpolates the absolute type trajectory basing on the list of poses of stored in objects of types derived from %trajectory_pose.
                         * @param pose_vector_iterator iterator to the list of positions
                         * @param coordinate_vector list of coordinates
                         * @param mc time of a single macrostep
                         * @return true if the interpolation was successful
                         */
                        virtual bool interpolate_absolute_pose(typename std::vector<Pos>::iterator & pose_vector_iterator, std::vector<std::vector<double> > & coordinate_vector, const double mc) = 0;

                        /**
                         * Method is used to interpolate the Angle Axis absolute pose, which was previously transformed into relative pose using the velocity_profile::calculate_relative_angle_axis_vector method (coordinates vector is now a relative vector).
                         * @param it iterator to the list of positions
                         * @param cv list of coordinates
                         * @param mc time of a single macrostep
                         * @return true if the interpolation was successful
                         */
                        bool interpolate_angle_axis_absolute_pose_transformed_into_relative(typename std::vector <Pos>::iterator & it, std::vector <std::vector <double> > & cv, const double mc) {

                            typename std::vector<double> coordinates(it->axes_num);

                            double start_position_array[6];

                            lib::Homog_matrix begining_frame;
                            lib::Homog_matrix goal_frame;
                            lib::Homog_matrix total_increment_frame;

                            lib::Xyz_Angle_Axis_vector total_angle_axis_increment_vector;
                            lib::Xyz_Angle_Axis_vector total_angle_axis_increment_vector_copy;

                            std::size_t z;

                            for (z = 0; z < 6; z++) {
                                total_angle_axis_increment_vector[z] = 0;
                            }

                            lib::Xyz_Angle_Axis_vector tmp_angle_axis_vector;

                            //printf("start pos:\t");
                            for (z = 0; z < 6; z++) {
                                start_position_array[z] = it->start_position[z];
                                //printf("%f\t", start_position_array[z]);
                            }
                            //printf("\n");

                            begining_frame.set_from_xyz_angle_axis(start_position_array);
                            goal_frame.set_from_xyz_angle_axis(start_position_array);

                            for (int i = 0; i < it->interpolation_node_no; i++) {
                            	//std::printf("coord %d:\t", i + 1);
                                for (std::size_t j = 0; j < it->axes_num; j++) {
                                    //if (fabs(it->s[j]) < 0.0000001) {
                                    //    coordinates[j] = 0;
                                    //    std::printf("%f\t", coordinates[j]);
                                    //} else {
                                        coordinates[j] = generate_relative_coordinate(i, it, j, mc);
                                        //std::printf("%f\t", coordinates[j]);
                                    //}
                                }
                                //std::printf("\n");

                                for (z = 0; z < 6; z++) {
                                    total_angle_axis_increment_vector[z] += coordinates[z];
                                }

                                //printf("taaiv:\t");
                                //for (z = 0; z < 6; z++) {
                                    //printf("%f\t", total_angle_axis_increment_vector[z]);
                                //}
                                //printf("\n");
                                total_angle_axis_increment_vector_copy = !it->xsi_star_matrix * total_angle_axis_increment_vector;

                                total_increment_frame.set_from_xyz_angle_axis(total_angle_axis_increment_vector_copy);

                                //przemnozyc kopie increment vectora razy macierz odwrotna do xsi * czyli !xsi*

                                goal_frame = begining_frame * total_increment_frame;

                                goal_frame.get_xyz_angle_axis(tmp_angle_axis_vector);
                                //printf("taav:\t");
                                //for (z = 0; z < 6; z++) {
                                 //   printf("%f\t", tmp_angle_axis_vector[z]);
                                //}
                                //printf("\n");
                                tmp_angle_axis_vector.to_vector(coordinates);

                                //printf("gener:\t");
                                //for (z = 0; z < 6; z++) {
                                //    printf("%f\t", coordinates[z]);
                                //}
                                //printf("\n");

                                //TODO add checking the correctness of the returned values

                                cv.push_back(coordinates);
                            }

                            return true;
                        }

                    protected:
                        /**
                         * Method generates a single relative type coordinate.
                         * @param node_counter number of current node (macrostep)
                         * @param it iterator to the list of positions
                         * @param axis_num number of current axis for which the calculations are performed
                         * @param mc time of a single macrostep
                         * @return single, generated coordinate
                         */
                        virtual double generate_relative_coordinate(int node_counter, typename std::vector <Pos>::iterator & it, int axis_num, const double mc) = 0;
                    };

                } // namespace trajectory_interpolator
            } // namespace generator
        } // namespace common
    } // namespace ecp
} // namespace mrrocpp

#endif /* _TRAJECTORY_INTERPOLATOR_H_ */
