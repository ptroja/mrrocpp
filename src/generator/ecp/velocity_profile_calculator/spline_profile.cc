/**
 * @file
 * @brief Contains definitions of the methods of spline_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#include "spline_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

using namespace std;

spline_profile::spline_profile()
{

}

spline_profile::~spline_profile()
{

}

bool spline_profile::calculate_time(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, int i)
{
    if (eq(it->s[i], 0.0) || eq(it->v_r[i], 0.0)) {//if distance to be covered or maximal velocity equal to 0
        it->times[i] = 0;
    } else {//normal calculation
        it->times[i] = it->s[i]/(it->v_r[i] * it->a_r[i]);
    }
    return true;

}

inline void spline_profile::generatePowers(int power, double x, double * powers)
{
  powers[0] = 1.0;
  for (int i = 1; i <= power; i++)
  {
    powers[i] = powers[i-1] * x;
  }
  return;
}

bool spline_profile::calculate_linear_coeffs(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i)
{

  if (it->t == 0)
  {
      return false;
  }

  if (it->t <= std::numeric_limits<double>::epsilon() )
  {
    it->coeffs[i][0] = it->start_position[i];
    it->coeffs[i][1] = 0.0;
    it->coeffs[i][2] = 0.0;
    it->coeffs[i][3] = 0.0;
    it->coeffs[i][4] = 0.0;
    it->coeffs[i][5] = 0.0;
  }
  else
  {
    it->coeffs[i][0] = it->start_position[i];
    it->coeffs[i][1] = (it->coordinates[i] - it->start_position[i]) / it->t;
    it->coeffs[i][2] = 0.0;
    it->coeffs[i][3] = 0.0;
    it->coeffs[i][4] = 0.0;
    it->coeffs[i][5] = 0.0;
  }

  return true;
}

bool spline_profile::calculate_cubic_coeffs(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i)
{
    if (it->t == 0)
    {
        return false;
    }

  double t[4];
  generatePowers(3, it->t, t);

  if (it->t <= std::numeric_limits<double>::epsilon() )
  {
    it->coeffs[i][0] = it->coordinates[i];
    it->coeffs[i][1] = it->v_k[i];
    it->coeffs[i][2] = 0.0;
    it->coeffs[i][3] = 0.0;
    it->coeffs[i][4] = 0.0;
    it->coeffs[i][5] = 0.0;
  }
  else
  {
    it->coeffs[i][0] = it->start_position[i];
    it->coeffs[i][1] = it->v_p[i];
    it->coeffs[i][2] = (-3.0*it->start_position[i] + 3.0*it->coordinates[i] - 2.0*it->v_p[i]*t[1] - it->v_k[i]*t[1]) / t[2];
    it->coeffs[i][3] = (2.0*it->start_position[i] - 2.0*it->coordinates[i] + it->v_p[i]*t[1] + it->v_k[i]*t[1]) / t[3];
    it->coeffs[i][4] = 0.0;
    it->coeffs[i][5] = 0.0;
  }

  return true;
}

bool spline_profile::calculate_quintic_coeffs(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i)
{

    if (it->t == 0)
    {
        return false;
    }

  double t[6];
  generatePowers(5, it->t, t);

  if (it->t <= std::numeric_limits<double>::epsilon() )
  {
    it->coeffs[i][0] = it->coordinates[i];
    it->coeffs[i][1] = it->v_k[i];
    it->coeffs[i][2] = 0.5 * it->a_k[i];
    it->coeffs[i][3] = 0.0;
    it->coeffs[i][4] = 0.0;
    it->coeffs[i][5] = 0.0;
  }
  else
  {
    it->coeffs[i][0] = it->start_position[i];
    it->coeffs[i][1] = it->v_p[i];
    it->coeffs[i][2] = 0.5*it->a_p[i];
    it->coeffs[i][3] = (-20.0*it->start_position[i] + 20.0*it->coordinates[i] - 3.0*it->a_p[i]*t[2] + it->a_k[i]*t[2] -
                       12.0*it->v_p[i]*t[1] - 8.0*it->v_k[i]*t[1]) / (2.0*t[3]);
    it->coeffs[i][4] = (30.0*it->start_position[i] - 30.0*it->coordinates[i] + 3.0*it->a_p[i]*t[2] - 2.0*it->v_k[i]*t[2] +
                       16.0*it->v_p[i]*t[1] + 14.0*it->v_k[i]*t[1]) / (2.0*t[4]);
    it->coeffs[i][5] = (-12.0*it->start_position[i] + 12.0*it->coordinates[i] - it->a_p[i]*t[2] + it->v_k[i]*t[2] -
                       6.0*it->v_p[i]*t[1] - 6.0*it->v_k[i]*t[1]) / (2.0*t[5]);
  }

  return true;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
