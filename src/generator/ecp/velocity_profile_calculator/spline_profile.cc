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
   // printf("calculate time\n");
    if (eq(it->s[i], 0.0) || eq(it->v_r[i], 0.0)) {//if distance to be covered or maximal velocity equal to 0
        it->times[i] = 0;
    } else {//normal calculation
        if (it->type == linear)
        {
            it->times[i] = it->s[i] / it->v_r[i];
        }
        else if (it->type == cubic)
        {
            it->times[i] = (it->s[i] / it->v_r[i]) * 1;//additional multiplier (just in case)
            //printf("times: %f\n", it->times[i]);
        }
        else if (it->type == quintic)
        {
            it->times[i] = (it->s[i] / it->v_r[i]) * 1;//additional multiplier (just in case)
        }
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

  vector<double> coefficients(6);
  it->coeffs[i] = coefficients;

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

  //printf("linear coeff 0: %f\n", it->coeffs[i][0]);
  //printf("linear coeff 1: %f\n", it->coeffs[i][1]);

  return true;
}

bool spline_profile::calculate_cubic_coeffs(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i)
{
    if (it->t == 0)
    {
        //printf("calculate cubic coeffs t: %f\n", it->t);
        //printf("calculate cubic coeffs pos_num: %d\n", it->pos_num);
        //flushall();
        return false;
    }

  double t[4];
  generatePowers(3, it->t, t);

  vector<double> coefficients(6);
  it->coeffs[i] = coefficients;

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

  vector<double> coefficients(6);
  it->coeffs[i] = coefficients;

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

bool spline_profile::set_v_p(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &beginning_it, int i)
{
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

bool spline_profile::set_v_p_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &beginning_it)
{
    bool trueFlag = true;

    for (int i = 0; i < it->axes_num; i++) {
            if (set_v_p(it, beginning_it, i) == false) {
                    trueFlag = false;
            }
    }
    //printf("\n");

    return trueFlag;
}

bool spline_profile::set_v_k(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & end_it, int i) {

        //printf("set_v_k\n");
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

bool spline_profile::set_v_k_pose(vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & end_it) {
        bool trueFlag = true;

        for (int i = 0; i < it->axes_num; i++) {
                if (set_v_k(it, end_it, i) == false) {
                        trueFlag = false;
                }
        }

        //printf("\n");
        return trueFlag;
}

bool spline_profile::set_a_p(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &beginning_it, int i)
{
    if (eq(it->s[i],0)) {
            it->a_p[i] = 0;
            //printf("a_p: %f\t", it->a_p[i]);
            return true;
    }

    if (it == beginning_it) {
            it->a_p[i] = 0;
    } else {
            it--;
            double temp_a_p = it->a_k[i];
            it++;
            it->a_p[i] = temp_a_p;
    }
    //printf("a_p: %f\t", it->a_p[i]);
    return true;
}

bool spline_profile::set_a_p_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &beginning_it)
{
    bool trueFlag = true;

    for (int i = 0; i < it->axes_num; i++) {
            if (set_a_p(it, beginning_it, i) == false) {
                    trueFlag = false;
            }
    }
    //printf("\n");

    return trueFlag;
}

bool spline_profile::set_a_k(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &end_it, int i)
{
    if (eq(it->s[i],0)) {
            it->a_k[i] = 0;
            //printf("a_k: %f\t", it->a_k[i]);
            return true;
    }

    double temp_k = it->k[i];
    it++;

    if (it == end_it) {
            it--;
            it->a_k[i] = 0;
    } else {
            if (temp_k == it->k[i]) {
                    double temp_a_k = it->a_r[i];
                    //printf("v_r of next pose: %f\t", it->v_r[i]);
                    it--;
                    if (temp_a_k > it->a_r[i]) {
                            it->a_k[i] = it->a_r[i];
                            //printf("temp_v_k: %f\t", temp_a_k);
                    } else {
                            it->a_k[i] = temp_a_k;
                            //printf("else temp_a_k: %f\t", temp_a_k);
                    }
            } else {
                    it--;
                    it->a_k[i] = 0;
            }
    }

    //printf("a_k: %f\n", it->a_k[i]);
    return true;
}

bool spline_profile::set_a_k_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator &end_it)
{
    bool trueFlag = true;

    for (int i = 0; i < it->axes_num; i++) {
            if (set_a_k(it, end_it, i) == false) {
                    trueFlag = false;
            }
    }

    //printf("\n");
    return trueFlag;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
