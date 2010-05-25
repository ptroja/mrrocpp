// -------------------------------------------------------------------------
//                             ecp_gen_test.cc
//             Effector Control Process (lib::ECP) - force & torque methods
// 			Test nowego EDP z wykorzystaniem sily
// 			By Slawek Bazant
//			Ostatnia modyfikacja: 05.01.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_g_dung.h"

#include "lib/mrmath/mrmath.h"
#include "math.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace generator {

dung::dung(common::task::task& _ecp_task, int step) :
	generator(_ecp_task)
{
    step_no = step;
    oq1 = oq2 = oq3 = oq4 = oq5 = oq6 = 200.0;
}

bool dung::first_step()
{
    td.interpolation_node_no = 1;
    td.internode_step_no = step_no;
    td.value_in_step_no = td.internode_step_no - 2;

    lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
    tool_frame.get_frame_tab(the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame);

    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
    the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
    the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
    the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
    the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
     the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
    the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
    the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

    return true;
}

bool dung::next_step ( )
{
    // define variable
    double	g[6];
    double	f[6];
    double	t[6];
    double	q1, q2, q3, q4, q5, q6;
    double	dq1, dq2, dq3, dq4, dq5, dq6;
    double	l1, l2, l3, l4, l5, l6, l7, l8, l9, l10, l11, l12, l13, l14, l15, l16, l17, l18, l19, l20, l21, l22, l23, l24, l25, l26, l27, l28, l29, l30;

    double	xx1, xy1, xz1, yy1, yz1, zz1;
    double	xx2, xy2, xz2, yy2, yz2, zz2;
    double	xx3, xy3, xz3, yy3, yz3, zz3;
    double	xx4, xy4, xz4, yy4, yz4, zz4;
    double	xx5, xy5, xz5, yy5, yz5, zz5;
    double	xx6, xy6, xz6, yy6, yz6, zz6;

    double	m1, m2, m3, m4, m5, m6;
    double	a2, a3, d5;
    double	xc2, xc3, zc5, zc6;

    double	M[6][6];
    double	J[6][6];

    //
    // static int count;
    // struct timespec start[9];
    if (check_and_null_trigger())
    {
        return false;
    }
    the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

    // DUNG START
    q1 = the_robot->reply_package.arm.pf_def.arm_coordinates[0];
    if (oq1 >= 100)
    {
        dq1 = 0.0;
        oq1 = q1;
    }
    else
    {
        dq1 = (q1-oq1)/0.01;
        oq1 = q1;
    }
    q2 = the_robot->reply_package.arm.pf_def.arm_coordinates[1];
    if (oq2 >= 100)
    {
        dq2 = 0.0;
        oq2 = q2;
    }
    else
    {
        dq2 = (q2-oq2)/0.01;
        oq2 = q2;
    }
    q3 = the_robot->reply_package.arm.pf_def.arm_coordinates[2] - the_robot->reply_package.arm.pf_def.arm_coordinates[1];
    if (oq3 >= 100)
    {
        dq3 = 0.0;
        oq3 = q3;
    }
    else
    {
        dq3 = (q3-oq3)/0.01;
        oq3 = q3;
    }
    q4 = the_robot->reply_package.arm.pf_def.arm_coordinates[3] - the_robot->reply_package.arm.pf_def.arm_coordinates[2] - M_PI_2;
    if (oq4 >= 100)
    {
        dq4 = 0.0;
        oq4 = q4;
    }
    else
    {
        dq4 = (q4-oq4)/0.01;
        oq4 = q4;
    }
    q5 = the_robot->reply_package.arm.pf_def.arm_coordinates[4];
    if (oq5 >= 100)
    {
        dq5 = 0.0;
        oq5 = q5;
    }
    else
    {
        dq5 = (q5-oq5)/0.01;
        oq5 = q5;
    }
    q6 = the_robot->reply_package.arm.pf_def.arm_coordinates[5];
    if (oq6 >= 100)
    {
        dq6 = 0.0;
        oq6 = q6;
    }
    else
    {
        dq6 = (q6-oq6)/0.01;
        oq6 = q6;
    }
    //printf("q1 = %f,    q2 = %f,    q3 = %f\n", q1*180/M_PI, q2*180/M_PI, q3*180/M_PI);
    // compute the jacobian
    /*		J[0][0] = -(sin(q1)*(a2*cos(q2) + a3*cos(q2 + q3) - d5*sin(q2 + q3 + q4)));
    		J[0][1] = -(cos(q1)*(d5*cos(q2 + q3 + q4) + a2*sin(q2) + a3*sin(q2 + q3)));
    		J[0][2] = -(cos(q1)*(d5*cos(q2 + q3 + q4) + a3*sin(q2 + q3)));
    		J[0][3] = -(d5*cos(q1)*cos(q2 + q3 + q4));
    		J[0][4] = 0;
    		J[0][5] = 0;
    		J[1][0] = cos(q1)*(a2*cos(q2) + a3*cos(q2 + q3) - d5*sin(q2 + q3 + q4));
    		J[1][1] = -(sin(q1)*(d5*cos(q2 + q3 + q4) + a2*sin(q2) + a3*sin(q2 + q3)));
    		J[1][2] = -(sin(q1)*(d5*cos(q2 + q3 + q4) + a3*sin(q2 + q3)));
    		J[1][3] = -(d5*cos(q2 + q3 + q4)*sin(q1));
    		J[1][4] = 0;
    		J[1][5] = 0;
    		J[2][0] = 0;
    		J[2][1] = -(a2*cos(q2)) - a3*cos(q2 + q3) + d5*sin(q2 + q3 + q4);
    		J[2][2] = -(a3*cos(q2 + q3)) + d5*sin(q2 + q3 + q4);
    		J[2][3] = d5*sin(q2 + q3 + q4);
    		J[2][4] = 0;
    		J[2][5] = 0;
    		J[3][0] = 0;
    		J[3][1] = -sin(q1);
    		J[3][2] = -sin(q1);
    		J[3][3] = -sin(q1);
    		J[3][4] = -(cos(q1)*sin(q2 + q3 + q4));
    		J[3][5] = -(cos(q5)*sin(q1)) + cos(q1)*cos(q2 + q3 + q4)*sin(q5);
    		J[4][0] = 0;
    		J[4][1] = cos(q1);
    		J[4][2] = cos(q1);
    		J[4][3] = cos(q2);
    		J[4][4] = -(sin(q1)*sin(q2 + q3 + q4));
    		J[4][5] = cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5);
    		J[5][0] = 1;
    		J[5][1] = 0;
    		J[5][2] = 0;
    		J[5][3] = 0;
    		J[5][4] = -cos(q2 + q3 + q4);
    		J[5][5] = -(sin(q2 + q3 + q4)*sin(q5));*/
    // gravity compensation
    g[0] = 0.0;
    g[1] = 9.8 * (17.57*cos(q2) - 0.343*cos(q2+q3) - 1.05*sin(q2+q3+q4) + 0.05*cos(q2+q3+q4)*sin(q5));
    g[2] = -9.8 * (0.34*cos(q2+q3) + 1.05*sin(q2+q3+q4) - 0.05*cos(q2+q3+q4)*sin(q5));
    g[3] = g[4] = g[5] = 0.0;
    //printf("q1 = %f,    q2 = %f,    q3 = %f,    q4 = %f,    q5 = %f,    q6 = %f,    g[2] = %f\n", q1*180/M_PI, q2*180/M_PI, q3*180/M_PI, q4*180/M_PI, q5*180/M_PI, q6*180/M_PI, g[1]);
    // compute the lump parameters
    /*		l1 = power(a3,2)*(m4 + m5 + m6) + power(a2,2)*(m3 + m4 + m5 + m6) + m2*power(xc2,2) + m3*power(xc3,2) + yy2 + yy3 + yy4 + zz1;
    		l2 = -(power(a2,2)*(m3 + m4 + m5 + m6)) - m2*power(xc2,2) + xx2 - yy2;
    		l3 = power(a2,2)*(m3 + m4 + m5 + m6) + m2*power(xc2,2) + zz2;
    		l4 = xy2;
    		l5 = xz2;
    		l6 = yz2;
    		l7 = a2*(m3 + m4 + m5 + m6) + m2*xc2;
    		l8 = -(power(a3,2)*(m4 + m5 + m6)) - m3*power(xc3,2) + xx3 - yy3;
    		l9 = power(a3,2)*(m4 + m5 + m6) + m3*power(xc3,2) + zz3;
    		l10 = xy3;
    		l11 = xz3;
    		l12 = yz3;
    		l13 = a3*(m4 + m5 + m6) + m3*xc3;
    		l14 = power(d5,2)*(m5 + m6) + xx4 - yy4 + yy5 + 2*d5*m5*zc5 + m5*power(zc5,2);
    		l15 = power(d5,2)*(m5 + m6) + yy5 + 2*d5*m5*zc5 + m5*power(zc5,2) + zz4;
    		l16 = xy4;
    		l17 = xz4;
    		l18 = yz4;
    		l19 = d5*(m5 + m6) + m5*zc5;
    		l20 = xx5 - yy5 + yy6 + m6*power(zc6,2);
    		l21 = yy6 + m6*power(zc6,2) + zz5;
    		l22 = xy5;
    		l23 = xz5;
    		l24 = yz5;
    		l25 = -(m6*zc6);
    		l26 = xx6 - yy6;
    		l27 = zz6;
    		l28 = xy6;
    		l29 = xz6;
    		l30 = yz6;
    		// compute mass matrix
    		M[0][0] =  (32*l1 + 16*l14 + 16*l2 + 8*l20 + 16*l21 + 12*l26 + 8*l27 + 16*l8 - 16*l2*cos(2*q2) + 32*a2*l13*cos(q3) - 16*l8*cos(2*(q2 + q3)) + 32*a2*l13*cos(2*q2 + q3) - 16*l14*cos(2*(q2 + q3 + q4)) - 8*l20*cos(2*(q2 + q3 + q4)) + 16*l21*cos(2*(q2 + q3 + q4)) + 4*l26*cos(2*(q2 + q3 + q4)) - 8*l27*cos(2*(q2 + q3 + q4)) - 4*l20*cos(2*(q2 + q3 + q4 - q5)) - 2*l26*cos(2*(q2 + q3 + q4 - q5)) + 4*l27*cos(2*(q2 + q3 + q4 - q5)) - 16*l24*cos(2*q2 + 2*q3 + 2*q4 - q5) + 16*d5*l25*cos(2*q2 + 2*q3 + 2*q4 - q5) + 8*l20*cos(2*q5) + 4*l26*cos(2*q5) - 8*l27*cos(2*q5) - 4*l20*cos(2*(q2 + q3 + q4 + q5)) - 2*l26*cos(2*(q2 + q3 + q4 + q5)) + 4*l27*cos(2*(q2 + q3 + q4 + q5)) + 16*l24*cos(2*q2 + 2*q3 + 2*q4 + q5) - 16*d5*l25*cos(2*q2 + 2*q3 + 2*q4 + q5) + 4*l26*cos(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6) + 4*l26*cos(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6) - 6*l26*cos(2*(q2 + q3 + q4 - q6)) - 4*l30*cos(2*q2 + 2*q3 + 2*q4 - 2*q5 - q6) - l26*cos(2*(q2 + q3 + q4 - q5 - q6)) + 8*l30*cos(2*q2 + 2*q3 + 2*q4 - q5 - q6) + 2*l26*cos(2*(q5 - q6)) - l26*cos(2*(q2 + q3 + q4 + q5 - q6)) - 8*l30*cos(2*q2 + 2*q3 + 2*q4 + q5 - q6) - 8*l30*cos(2*q5 - q6) + 4*l30*cos(2*q2 + 2*q3 + 2*q4 + 2*q5 - q6) - 4*l26*cos(2*q6) - 6*l26*cos(2*(q2 + q3 + q4 + q6)) + 4*l30*cos(2*q2 + 2*q3 + 2*q4 - 2*q5 + q6) - l26*cos(2*(q2 + q3 + q4 - q5 + q6)) + 8*l30*cos(2*q2 + 2*q3 + 2*q4 - q5 + q6) + 2*l26*cos(2*(q5 + q6)) - l26*cos(2*(q2 + q3 + q4 + q5 + q6)) - 8*l30*cos(2*q2 + 2*q3 + 2*q4 + q5 + q6) + 8*l30*cos(2*q5 + q6) - 4*l30*cos(2*q2 + 2*q3 + 2*q4 + 2*q5 + q6) - 4*l26*cos(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6) - 4*l26*cos(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6) + 32*l4*sin(2*q2) + 32*l10*sin(2*(q2 + q3)) - 32*a3*l19*sin(q4) - 32*a2*l19*sin(q3 + q4) + 32*l16*sin(2*(q2 + q3 + q4)) - 32*a2*l19*sin(2*q2 + q3 + q4) - 32*a3*l19*sin(2*q2 + 2*q3 + q4) + 16*a3*l25*sin(q4 - q5) + 16*a2*l25*sin(q3 + q4 - q5) - 8*l22*sin(2*(q2 + q3 + q4 - q5)) + 16*a2*l25*sin(2*q2 + q3 + q4 - q5) + 16*a3*l25*sin(2*q2 + 2*q3 + q4 - q5) + 16*l23*sin(2*q2 + 2*q3 + 2*q4 - q5) - 16*l22*sin(2*q5) - 16*a3*l25*sin(q4 + q5) - 16*a2*l25*sin(q3 + q4 + q5) + 8*l22*sin(2*(q2 + q3 + q4 + q5)) - 16*a2*l25*sin(2*q2 + q3 + q4 + q5) - 16*a3*l25*sin(2*q2 + 2*q3 + q4 + q5) + 16*l23*sin(2*q2 + 2*q3 + 2*q4 + q5) + 8*l28*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6) + 8*l28*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6) - 12*l28*sin(2*(q2 + q3 + q4 - q6)) + 4*l29*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - q6) - 2*l28*sin(2*(q2 + q3 + q4 - q5 - q6)) - 8*l29*sin(2*q2 + 2*q3 + 2*q4 - q5 - q6) + 4*l28*sin(2*(q5 - q6)) - 2*l28*sin(2*(q2 + q3 + q4 + q5 - q6)) + 8*l29*sin(2*q2 + 2*q3 + 2*q4 + q5 - q6) + 8*l29*sin(2*q5 - q6) - 4*l29*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - q6) + 8*l28*sin(2*q6) + 12*l28*sin(2*(q2 + q3 + q4 + q6)) + 4*l29*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + q6) + 2*l28*sin(2*(q2 + q3 + q4 - q5 + q6)) + 8*l29*sin(2*q2 + 2*q3 + 2*q4 - q5 + q6) - 4*l28*sin(2*(q5 + q6)) + 2*l28*sin(2*(q2 + q3 + q4 + q5 + q6)) - 8*l29*sin(2*q2 + 2*q3 + 2*q4 + q5 + q6) + 8*l29*sin(2*q5 + q6) - 4*l29*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + q6) + 8*l28*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6) + 8*l28*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/32.;
    		M[0][1] = (-16*l6*cos(q2) - 16*l12*cos(q2 + q3) - 16*l18*cos(q2 + q3 + q4) + 4*l20*cos(q2 + q3 + q4 - 2*q5) + 2*l26*cos(q2 + q3 + q4 - 2*q5) - 4*l27*cos(q2 + q3 + q4 - 2*q5) + 8*l24*cos(q2 + q3 + q4 - q5) - 8*d5*l25*cos(q2 + q3 + q4 - q5) + 8*l24*cos(q2 + q3 + q4 + q5) - 8*d5*l25*cos(q2 + q3 + q4 + q5) - 4*l20*cos(q2 + q3 + q4 + 2*q5) - 2*l26*cos(q2 + q3 + q4 + 2*q5) + 4*l27*cos(q2 + q3 + q4 + 2*q5) + l26*cos(q2 + q3 + q4 - 2*q5 - 2*q6) - 2*l26*cos(q2 + q3 + q4 - q5 - 2*q6) + 2*l26*cos(q2 + q3 + q4 + q5 - 2*q6) - l26*cos(q2 + q3 + q4 + 2*q5 - 2*q6) + 4*l30*cos(q2 + q3 + q4 - 2*q5 - q6) - 4*l30*cos(q2 + q3 + q4 - q5 - q6) - 4*l30*cos(q2 + q3 + q4 + q5 - q6) + 4*l30*cos(q2 + q3 + q4 + 2*q5 - q6) - 4*l30*cos(q2 + q3 + q4 - 2*q5 + q6) - 4*l30*cos(q2 + q3 + q4 - q5 + q6) - 4*l30*cos(q2 + q3 + q4 + q5 + q6) - 4*l30*cos(q2 + q3 + q4 + 2*q5 + q6) + l26*cos(q2 + q3 + q4 - 2*q5 + 2*q6) + 2*l26*cos(q2 + q3 + q4 - q5 + 2*q6) - 2*l26*cos(q2 + q3 + q4 + q5 + 2*q6) - l26*cos(q2 + q3 + q4 + 2*q5 + 2*q6) - 16*l5*sin(q2) - 16*l11*sin(q2 + q3) - 16*l17*sin(q2 + q3 + q4) + 8*l22*sin(q2 + q3 + q4 - 2*q5) - 8*a2*l25*sin(q2 - q5) - 8*a3*l25*sin(q2 + q3 - q5) - 8*l23*sin(q2 + q3 + q4 - q5) - 8*a2*l25*sin(q2 + q5) - 8*a3*l25*sin(q2 + q3 + q5) + 8*l23*sin(q2 + q3 + q4 + q5) + 8*l22*sin(q2 + q3 + q4 + 2*q5) + 2*l28*sin(q2 + q3 + q4 - 2*q5 - 2*q6) - 4*l28*sin(q2 + q3 + q4 - q5 - 2*q6) + 4*l28*sin(q2 + q3 + q4 + q5 - 2*q6) - 2*l28*sin(q2 + q3 + q4 + 2*q5 - 2*q6) - 4*l29*sin(q2 + q3 + q4 - 2*q5 - q6) + 4*l29*sin(q2 + q3 + q4 - q5 - q6) + 4*l29*sin(q2 + q3 + q4 + q5 - q6) - 4*l29*sin(q2 + q3 + q4 + 2*q5 - q6) - 4*l29*sin(q2 + q3 + q4 - 2*q5 + q6) - 4*l29*sin(q2 + q3 + q4 - q5 + q6) - 4*l29*sin(q2 + q3 + q4 + q5 + q6) - 4*l29*sin(q2 + q3 + q4 + 2*q5 + q6) - 2*l28*sin(q2 + q3 + q4 - 2*q5 + 2*q6) - 4*l28*sin(q2 + q3 + q4 - q5 + 2*q6) + 4*l28*sin(q2 + q3 + q4 + q5 + 2*q6) + 2*l28*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16.;
    		M[1][0] = M[0][1];
    		M[0][2] = (-16*l12*cos(q2 + q3) - 16*l18*cos(q2 + q3 + q4) + 4*l20*cos(q2 + q3 + q4 - 2*q5) + 2*l26*cos(q2 + q3 + q4 - 2*q5) - 4*l27*cos(q2 + q3 + q4 - 2*q5) + 8*l24*cos(q2 + q3 + q4 - q5) - 8*d5*l25*cos(q2 + q3 + q4 - q5) + 8*l24*cos(q2 + q3 + q4 + q5) - 8*d5*l25*cos(q2 + q3 + q4 + q5) - 4*l20*cos(q2 + q3 + q4 + 2*q5) - 2*l26*cos(q2 + q3 + q4 + 2*q5) + 4*l27*cos(q2 + q3 + q4 + 2*q5) + l26*cos(q2 + q3 + q4 - 2*q5 - 2*q6) - 2*l26*cos(q2 + q3 + q4 - q5 - 2*q6) + 2*l26*cos(q2 + q3 + q4 + q5 - 2*q6) - l26*cos(q2 + q3 + q4 + 2*q5 - 2*q6) + 4*l30*cos(q2 + q3 + q4 - 2*q5 - q6) - 4*l30*cos(q2 + q3 + q4 - q5 - q6) - 4*l30*cos(q2 + q3 + q4 + q5 - q6) + 4*l30*cos(q2 + q3 + q4 + 2*q5 - q6) - 4*l30*cos(q2 + q3 + q4 - 2*q5 + q6) - 4*l30*cos(q2 + q3 + q4 - q5 + q6) - 4*l30*cos(q2 + q3 + q4 + q5 + q6) - 4*l30*cos(q2 + q3 + q4 + 2*q5 + q6) + l26*cos(q2 + q3 + q4 - 2*q5 + 2*q6) + 2*l26*cos(q2 + q3 + q4 - q5 + 2*q6) - 2*l26*cos(q2 + q3 + q4 + q5 + 2*q6) - l26*cos(q2 + q3 + q4 + 2*q5 + 2*q6) - 16*l11*sin(q2 + q3) - 16*l17*sin(q2 + q3 + q4) + 8*l22*sin(q2 + q3 + q4 - 2*q5) - 8*a3*l25*sin(q2 + q3 - q5) - 8*l23*sin(q2 + q3 + q4 - q5) - 8*a3*l25*sin(q2 + q3 + q5) + 8*l23*sin(q2 + q3 + q4 + q5) + 8*l22*sin(q2 + q3 + q4 + 2*q5) + 2*l28*sin(q2 + q3 + q4 - 2*q5 - 2*q6) - 4*l28*sin(q2 + q3 + q4 - q5 - 2*q6) + 4*l28*sin(q2 + q3 + q4 + q5 - 2*q6) - 2*l28*sin(q2 + q3 + q4 + 2*q5 - 2*q6) - 4*l29*sin(q2 + q3 + q4 - 2*q5 - q6) + 4*l29*sin(q2 + q3 + q4 - q5 - q6) + 4*l29*sin(q2 + q3 + q4 + q5 - q6) - 4*l29*sin(q2 + q3 + q4 + 2*q5 - q6) - 4*l29*sin(q2 + q3 + q4 - 2*q5 + q6) - 4*l29*sin(q2 + q3 + q4 - q5 + q6) - 4*l29*sin(q2 + q3 + q4 + q5 + q6) - 4*l29*sin(q2 + q3 + q4 + 2*q5 + q6) - 2*l28*sin(q2 + q3 + q4 - 2*q5 + 2*q6) - 4*l28*sin(q2 + q3 + q4 - q5 + 2*q6) + 4*l28*sin(q2 + q3 + q4 + q5 + 2*q6) + 2*l28*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16.;
    		M[2][0] = M[0][2];
    		M[0][3] = (-16*l18*cos(q2 + q3 + q4) + 2*(2*l20 + l26 - 2*l27)*cos(q2 + q3 + q4 - 2*q5) + 8*l24*cos(q2 + q3 + q4 - q5) - 8*d5*l25*cos(q2 + q3 + q4 - q5) + 8*l24*cos(q2 + q3 + q4 + q5) - 8*d5*l25*cos(q2 + q3 + q4 + q5) - 4*l20*cos(q2 + q3 + q4 + 2*q5) - 2*l26*cos(q2 + q3 + q4 + 2*q5) + 4*l27*cos(q2 + q3 + q4 + 2*q5) + l26*cos(q2 + q3 + q4 - 2*q5 - 2*q6) - 2*l26*cos(q2 + q3 + q4 - q5 - 2*q6) + 2*l26*cos(q2 + q3 + q4 + q5 - 2*q6) - l26*cos(q2 + q3 + q4 + 2*q5 - 2*q6) + 4*l30*cos(q2 + q3 + q4 - 2*q5 - q6) - 4*l30*cos(q2 + q3 + q4 - q5 - q6) - 4*l30*cos(q2 + q3 + q4 + q5 - q6) + 4*l30*cos(q2 + q3 + q4 + 2*q5 - q6) - 4*l30*cos(q2 + q3 + q4 - 2*q5 + q6) - 4*l30*cos(q2 + q3 + q4 - q5 + q6) - 4*l30*cos(q2 + q3 + q4 + q5 + q6) - 4*l30*cos(q2 + q3 + q4 + 2*q5 + q6) + l26*cos(q2 + q3 + q4 - 2*q5 + 2*q6) + 2*l26*cos(q2 + q3 + q4 - q5 + 2*q6) - 2*l26*cos(q2 + q3 + q4 + q5 + 2*q6) - l26*cos(q2 + q3 + q4 + 2*q5 + 2*q6) - 16*l17*sin(q2 + q3 + q4) + 8*l22*sin(q2 + q3 + q4 - 2*q5) - 8*l23*sin(q2 + q3 + q4 - q5) + 8*l23*sin(q2 + q3 + q4 + q5) + 8*l22*sin(q2 + q3 + q4 + 2*q5) + 2*l28*sin(q2 + q3 + q4 - 2*q5 - 2*q6) - 4*l28*sin(q2 + q3 + q4 - q5 - 2*q6) + 4*l28*sin(q2 + q3 + q4 + q5 - 2*q6) - 2*l28*sin(q2 + q3 + q4 + 2*q5 - 2*q6) - 4*l29*sin(q2 + q3 + q4 - 2*q5 - q6) + 4*l29*sin(q2 + q3 + q4 - q5 - q6) + 4*l29*sin(q2 + q3 + q4 + q5 - q6) - 4*l29*sin(q2 + q3 + q4 + 2*q5 - q6) - 4*l29*sin(q2 + q3 + q4 - 2*q5 + q6) - 4*l29*sin(q2 + q3 + q4 - q5 + q6) - 4*l29*sin(q2 + q3 + q4 + q5 + q6) - 4*l29*sin(q2 + q3 + q4 + 2*q5 + q6) - 2*l28*sin(q2 + q3 + q4 - 2*q5 + 2*q6) - 4*l28*sin(q2 + q3 + q4 - q5 + 2*q6) + 4*l28*sin(q2 + q3 + q4 + q5 + 2*q6) + 2*l28*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16.;
    		M[3][0] = M[0][3];
    		M[0][4] = (-4*(2*l21 + l26)*cos(q2 + q3 + q4) + 4*(l24 - d5*l25)*cos(q2 + q3 + q4 - q5) - 4*l24*cos(q2 + q3 + q4 + q5) + 4*d5*l25*cos(q2 + q3 + q4 + q5) + 2*l26*cos(q2 + q3 + q4 - 2*q6) - l26*cos(q2 + q3 + q4 - q5 - 2*q6) - l26*cos(q2 + q3 + q4 + q5 - 2*q6) - 2*l30*cos(q2 + q3 + q4 - q5 - q6) + 2*l30*cos(q2 + q3 + q4 + q5 - q6) - 2*l30*cos(q2 + q3 + q4 - q5 + q6) + 2*l30*cos(q2 + q3 + q4 + q5 + q6) + 2*l26*cos(q2 + q3 + q4 + 2*q6) + l26*cos(q2 + q3 + q4 - q5 + 2*q6) + l26*cos(q2 + q3 + q4 + q5 + 2*q6) - 4*a2*l25*sin(q2 - q5) - 4*a3*l25*sin(q2 + q3 - q5) - 4*l23*sin(q2 + q3 + q4 - q5) + 4*a2*l25*sin(q2 + q5) + 4*a3*l25*sin(q2 + q3 + q5) - 4*l23*sin(q2 + q3 + q4 + q5) + 4*l28*sin(q2 + q3 + q4 - 2*q6) - 2*l28*sin(q2 + q3 + q4 - q5 - 2*q6) - 2*l28*sin(q2 + q3 + q4 + q5 - 2*q6) + 2*l29*sin(q2 + q3 + q4 - q5 - q6) - 2*l29*sin(q2 + q3 + q4 + q5 - q6) - 2*l29*sin(q2 + q3 + q4 - q5 + q6) + 2*l29*sin(q2 + q3 + q4 + q5 + q6) - 4*l28*sin(q2 + q3 + q4 + 2*q6) - 2*l28*sin(q2 + q3 + q4 - q5 + 2*q6) - 2*l28*sin(q2 + q3 + q4 + q5 + 2*q6))/8.;
    		M[4][0] = M[0][4];
    		M[0][5] = (-2*l27*cos(q2 + q3 + q4 - q5) + 2*l27*cos(q2 + q3 + q4 + q5) - 2*l30*cos(q2 + q3 + q4 - q6) + l30*cos(q2 + q3 + q4 - q5 - q6) + l30*cos(q2 + q3 + q4 + q5 - q6) - 2*l30*cos(q2 + q3 + q4 + q6) - l30*cos(q2 + q3 + q4 - q5 + q6) - l30*cos(q2 + q3 + q4 + q5 + q6) + 2*l29*sin(q2 + q3 + q4 - q6) - l29*sin(q2 + q3 + q4 - q5 - q6) - l29*sin(q2 + q3 + q4 + q5 - q6) - 2*l29*sin(q2 + q3 + q4 + q6) - l29*sin(q2 + q3 + q4 - q5 + q6) - l29*sin(q2 + q3 + q4 + q5 + q6))/4.;
    		M[5][0] = M[0][5];
    		M[1][1] = l15 + l3 + l9 + l27*power(cos(q5),2) - 2*a2*l19*cos(q4)*sin(q3) - 2*a3*l19*sin(q4) - 2*a3*l25*cos(q4)*sin(q5) - 2*l29*cos(q5)*cos(q6)*sin(q5) + 2*a2*l25*sin(q3)*sin(q4)*sin(q5) + l20*power(sin(q5),2) + l26*power(cos(q6),2)*power(sin(q5),2) + 2*a2*cos(q3)*(l13 - l19*sin(q4) - l25*cos(q4)*sin(q5)) + l22*sin(2*q5) - 2*l28*cos(q6)*power(sin(q5),2)*sin(q6) + l30*sin(2*q5)*sin(q6);
    		M[1][2] = l15 + l9 + l27*power(cos(q5),2) - a2*l19*cos(q4)*sin(q3) - 2*a3*l19*sin(q4) - 2*a3*l25*cos(q4)*sin(q5) - 2*l29*cos(q5)*cos(q6)*sin(q5) + a2*l25*sin(q3)*sin(q4)*sin(q5) + l20*power(sin(q5),2) + l26*power(cos(q6),2)*power(sin(q5),2) + a2*cos(q3)*(l13 - l19*sin(q4) - l25*cos(q4)*sin(q5)) + l22*sin(2*q5) - 2*l28*cos(q6)*power(sin(q5),2)*sin(q6) + l30*sin(2*q5)*sin(q6);
    		M[2][1] = M[1][2];
    		M[1][3] = l15 + l27*power(cos(q5),2) - a3*l19*sin(q4) - a2*l19*cos(q3)*sin(q4) - 2*l29*cos(q5)*cos(q6)*sin(q5) + a2*l25*sin(q3)*sin(q4)*sin(q5) + l20*power(sin(q5),2) + l26*power(cos(q6),2)*power(sin(q5),2) - cos(q4)*(a2*l19*sin(q3) + l25*(a3 + a2*cos(q3))*sin(q5)) + l22*sin(2*q5) - 2*l28*cos(q6)*power(sin(q5),2)*sin(q6) + l30*sin(2*q5)*sin(q6);
    		M[3][1] = M[1][3];
    		M[1][4] = cos(q5)*(-l24 + d5*l25 + l30*cos(q6) - a3*l25*sin(q4) - a2*l25*sin(q3 + q4) + l29*sin(q6)) - (sin(q5)*(2*l23 + 2*l28*cos(2*q6) + l26*sin(2*q6)))/2.;
    		M[4][1] = M[1][4];
    		M[1][5] = l27*cos(q5) + sin(q5)*(-(l29*cos(q6)) + l30*sin(q6));
    		M[5][1] = M[1][5];
    		M[2][2] = l15 + l9 + l27*power(cos(q5),2) - 2*a3*l19*sin(q4) - 2*a3*l25*cos(q4)*sin(q5) - 2*l29*cos(q5)*cos(q6)*sin(q5) + l20*power(sin(q5),2) + l26*power(cos(q6),2)*power(sin(q5),2) + l22*sin(2*q5) - 2*l28*cos(q6)*power(sin(q5),2)*sin(q6) + l30*sin(2*q5)*sin(q6);
    		M[2][3] = l15 + l27*power(cos(q5),2) - a3*l19*sin(q4) - a3*l25*cos(q4)*sin(q5) - 2*l29*cos(q5)*cos(q6)*sin(q5) + l20*power(sin(q5),2) + l26*power(cos(q6),2)*power(sin(q5),2) + l22*sin(2*q5) - 2*l28*cos(q6)*power(sin(q5),2)*sin(q6) + l30*sin(2*q5)*sin(q6);
    		M[3][2] = M[2][3];
    		M[2][4] = cos(q5)*(-l24 + d5*l25 + l30*cos(q6) - a3*l25*sin(q4) + l29*sin(q6)) - (sin(q5)*(2*l23 + 2*l28*cos(2*q6) + l26*sin(2*q6)))/2.;
    		M[4][2] = M[2][4];
    		M[2][5] = l27*cos(q5) + sin(q5)*(-(l29*cos(q6)) + l30*sin(q6));
    		M[5][2] = M[2][5];
    		M[3][3] = l15 + l27*power(cos(q5),2) - 2*l29*cos(q5)*cos(q6)*sin(q5) + l22*sin(2*q5) + l30*sin(2*q5)*sin(q6) + power(sin(q5),2)*(l20 + l26*power(cos(q6),2) - 2*l28*cos(q6)*sin(q6));
    		M[3][4] = cos(q5)*(-l24 + d5*l25 + l30*cos(q6) + l29*sin(q6)) - (sin(q5)*(2*l23 + 2*l28*cos(2*q6) + l26*sin(2*q6)))/2.;
    		M[4][3] = M[3][4];
    		M[3][5] = l27*cos(q5) + sin(q5)*(-(l29*cos(q6)) + l30*sin(q6));
    		M[5][3] = M[3][5];
    		M[4][4] = l21 + l26*power(sin(q6),2) + l28*sin(2*q6);
    		M[4][5] = l30*cos(q6) + l29*sin(q6);
    		M[5][4] = M[4][5];
    		M[5][5] = l27;*/
    // compute lamda = inverse(jacobian * inverse(mass) * transpose(jacobian));

    // compute the operational point position from joint position: X = X(q)

    // compute the operational point velocity: dX = J*q

    // compute the motion controller: Fmotion = lamda*(-Kv*dX + Kp*(Xdesired - X))

    // the below code is for friction compensation only
    f[0] = g[0] + (-500)*(0 - dq1) + 0*(0 - q1);
    f[1] = g[1] + (300)*(0 - dq2) + 0*(0 - q2);
    f[2] = g[2] + (-450)*(0 - dq3) + 0*(0 - q3);
    f[3] = g[3] + 0.0;
    f[4] = g[4] + 0.0;
    f[5] = g[5] + 0.0;
    // convert to torque: T = transpose(J)*Fmotion

    // below is just directly output to torque
    t[0] = f[0];
    t[1] = f[1];
    t[2] = f[2];
    t[3] = f[3];
    t[4] = f[4];
    t[5] = f[5];
    // the torque is then added with g. Note that here, we ignore the centrifugal and coriolis force!!!
    // here we ignore the centrifugal and corriolis force
    /*t[0] = t[0] + g[0];
    t[1] = t[1] + g[1];
    t[2] = t[2] + g[2];
    t[3] = t[3] + g[3];
    t[4] = t[4] + g[4];
    t[5] = t[5] + g[5];*/
    // output the torque to the controller
    for (int i=0; i<3; i++)
    {
        the_robot->ecp_command.instruction.arm.pf_def.desired_torque[i] = t[i];
    } // end:for

    for (int i=0; i<MAX_SERVOS_NR; i++)
    {
        the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] =
            the_robot->reply_package.arm.pf_def.arm_coordinates[i];
    } // end:for

    // DUNG STOP

    //	else the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate-0.0001;


    return true;
}

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

