/**
 * @file
 * @brief Contains definitions of the methods of spline_interpolator class.
 * @author rtulwin
 * @ingroup generators
 */

#include "spline_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

using namespace std;

spline_interpolator::spline_interpolator()
{
    // TODO Auto-generated constructor stub
}

spline_interpolator::~spline_interpolator()
{
    // TODO Auto-generated destructor stub
}

bool spline_interpolator::interpolate_relative_pose(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {

    vector<double> coordinates (it->axes_num);
    coordinates = it->start_position;
    for (int i = 0; i < it->interpolation_node_no; i++) {
        //printf("inter: \n");
            for (int j = 0; j < it->axes_num; j++) {
                coordinates[j] = calculate_velocity(it, j, (i+1) * mc) * mc;
                /*if (i == 0)
                {
                    coordinates[j] = calculate_position(it, j, (i+1) * mc);
                    printf("%f\t",coordinates[j]);
                }
                else
                {
                    coordinates[j] = calculate_position(it, j, (i+1) * mc) - cv[cv.size()-1][j];
                    printf("%f\t",calculate_position(it, j, (i+1) * mc));
                }*/
            }
            //printf("\n");
            cv.push_back(coordinates);
    }

    return true;
}

bool spline_interpolator::interpolate_absolute_pose(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, vector<vector<double> > & cv, const double mc) {

    vector<double> coordinates (it->axes_num);
    coordinates = it->start_position;
    for (int i = 0; i < it->interpolation_node_no; i++) {
            for (int j = 0; j < it->axes_num; j++) {
                coordinates[j] = calculate_position(it, j, (i+1) * mc);
            }
            cv.push_back(coordinates);
    }

    return true;
}

double spline_interpolator::generate_relative_coordinate(int node_counter, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, int axis_num, double mc)
{

    return calculate_velocity(it, axis_num, node_counter * mc) * mc;
}

double spline_interpolator::calculate_position(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time)
{
  double t[6];
  double position;
  generatePowers(5, time, t);

  position = t[0]*it->coeffs[i][0] +
             t[1]*it->coeffs[i][1] +
             t[2]*it->coeffs[i][2] +
             t[3]*it->coeffs[i][3] +
             t[4]*it->coeffs[i][4] +
             t[5]*it->coeffs[i][5];
  return position;
}

double spline_interpolator::calculate_velocity(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time)
{
  double t[5];
  double velocity;
  generatePowers(4, time, t);

  velocity = t[0]*it->coeffs[i][1] +
             2.0*t[1]*it->coeffs[i][2] +
             3.0*t[2]*it->coeffs[i][3] +
             4.0*t[3]*it->coeffs[i][4] +
             5.0*t[4]*it->coeffs[i][5];
  return velocity;
}

double spline_interpolator::calculate_acceleration(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i, double time)
{
  double t[4];
  double acceleration;
  generatePowers(3, time, t);

  acceleration = 2.0*t[0]*it->coeffs[i][2] +
                 6.0*t[1]*it->coeffs[i][3] +
                 12.0*t[2]*it->coeffs[i][4] +
                 20.0*t[3]*it->coeffs[i][5];
  return acceleration;
}

inline void spline_interpolator::generatePowers(int power, double x, double * powers)
{
  powers[0] = 1.0;
  for (int i = 1; i <= power; i++)
  {
    powers[i] = powers[i-1] * x;
  }
  return;
}

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
