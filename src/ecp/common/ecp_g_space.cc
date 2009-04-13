#include <stdio.h>
#include <math.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_space.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {


// ########################################################################################################
// ########################################################################################################
// ############## Definnitions for methods of irp6ot_hermite_spline_generator #############################
// ########################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------irp6ot_hermite_spline_generator - constuctor ---------------------------
// ----------------------------------------------------------------------------------------------
hermite_spline::hermite_spline (common::task::ecp_task& _ecp_task,
        double interval,double ts ) : ecp_teach_in_generator (_ecp_task)
{
    TSTEP=interval;
    INTERVAL=interval;
    TS=ts;
    int i,j;

    for (i=0;i<MAX_SERVOS_NR;i++)
    {				// setting the values of the default velocities
        v_def_motor[i]=90;
        v_def_joint[i]=50;
        v_def_xyz_angles[i]=50;
        for (j=0;j<NPOINTS;j++)
        {
            yi[j][i]=0;
        }
    }
    v_def_xyz_euler[0]=0.05;
    v_def_xyz_euler[1]=0.03;
    v_def_xyz_euler[2]=0.05;
    v_def_xyz_euler[3]=0.3;
    v_def_xyz_euler[4]=20;
    v_def_xyz_euler[5]=20;
    v_def_xyz_euler[6]=20;
    v_def_xyz_euler[7]=20;
}
; // end: irp6ot_hermite_spline_generator::irp6ot_hermite_spline_generator()

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda fill_hermite_arrays ----------------------------
// ----------------------------------------------------------------------------------------------
void hermite_spline::fill_hermite_arrays(void)
{
    if (pose_list.empty())
        throw ECP_error (NON_FATAL_ERROR, NON_COMPATIBLE_LISTS);
    int i,j;
    double dy1[MAX_SERVOS_NR],dy2[MAX_SERVOS_NR],dtime[MAX_SERVOS_NR],h1,h2,mi,la;        // aux variables
    npoints = pose_list_length() + 1;				// number of poses - the one additional is for the starting pose
    for (i=0;i<MAX_SERVOS_NR;i++)
        yi[0][i]=starting_pose.coordinates[i]; // Copy the values of the coordinates of the current robot pose from the virtual image of the robot to the first entry in the array yi
    initiate_pose_list();  // reset list pointers
    j=1;

    while (pose_list_iterator != pose_list.end()) // Copy the values of the coordinates of the control poses from the list pose_list_ptr to the array yi
    {
        for (i=0;i<MAX_SERVOS_NR;i++)
            yi[j][i]=pose_list_iterator->coordinates[i];
        // printf("\n fill arrays : yi[%d][2]  %f\n",i,yi[i][2]);
        pose_list_iterator++;
        j++;
    }


    // ################### calculating times between control poses - filling 'time'
    time[0]=0;
    for (j=1;j<=npoints;j++)
    {
        time[j]=time[j-1];
        switch ( starting_pose.arm_type )
        {
        case MOTOR:
            for(i=0;i<MAX_SERVOS_NR;i++)
            {
                dtime[i]=TS*(ceil((fabs(yi[j][i]-yi[j-1][i])/v_def_motor[i])/INTERVAL)*INTERVAL);
                if(dtime[i]<TS*0.1)
                    dtime[i]=TS*0.1;
                if (time[j]<time[j-1]+dtime[i])
                    time[j]=time[j-1]+dtime[i];
                // printf("\n   %f",dtime[i]);
            }
            break;
        case JOINT:
            for(i=0;i<MAX_SERVOS_NR;i++)
            {
                dtime[i]=TS*(ceil((fabs(yi[j][i]-yi[j-1][i])/v_def_joint[i])/INTERVAL)*INTERVAL);
                if(dtime[i]<TS*0.1)
                    dtime[i]=TS*0.1;
                if (time[j]<time[j-1]+dtime[i])
                    time[j]=time[j-1]+dtime[i];
                // printf("\n   %f",dtime[i]);
            }
            break;
        case XYZ_EULER_ZYZ:
            for(i=0;i<7;i++)
            {
                dtime[i]=TS*(ceil((fabs(yi[j][i]-yi[j-1][i])/v_def_xyz_euler[i])/INTERVAL)*INTERVAL);
                if(dtime[i]<TS*0.1)
                    dtime[i]=TS*0.1;
                if (time[j]<time[j-1]+dtime[i])
                    time[j]=time[j-1]+dtime[i];
                // printf("\n   %f",dtime[i]);
            }
            break;
        case XYZ_ANGLE_AXIS:
            for(i=0;i<7;i++)
            {
                dtime[i]=TS*(ceil((fabs(yi[j][i]-yi[j-1][i])/v_def_xyz_angles[i])/INTERVAL)*INTERVAL);
                if(dtime[i]<TS*0.1)
                    dtime[i]=TS*0.1;
                if(time[j]<time[j-1]+dtime[i])
                    time[j]=time[j-1]+dtime[i];
                // printf("\n   %f",dtime[i]);
            }
        default:
            throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
        } // end: switch   */
        // printf("\ntime[%d] %f", j,time[j]);
    }
    // #####################

    // Filling 'yiprim'
    for (j=1;j<npoints;j++)
        for (i=0;i<6;i++)
        {
            h1=time[j]-time[j-1];
            h2=time[j+1]-time[j];
            mi=h1/(h1+h2);
            la=1-mi;
            dy1[i] = yi[j][i] - yi[j-1][i];
            dy2[i] = yi[j+1][i] - yi[j][i];
            yiprim[j][i] = la * dy1[i]/h1 + mi * dy2[i]/h2;
            if(j==1)
                yiprim[0][i]= 0; // 1.5 * dy1[j] - 0.5 * dy2[j];
            if(j==npoints-1)
                yiprim[npoints][i]= 0; // -0.5 * dy1[j] + 1.5 * dy2[j];
        }

}
;// end: bool irp6ot_hermite_spline_generator::fill_hermite_arrays(void)

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda   calc_hermit -------------------------------------------------
// ----------------------------------------------------------------------------------------------

void hermite_spline::calc_hermit()
{
    int i,j;

    double A[MAX_SERVOS_NR],
    B[MAX_SERVOS_NR],
    dy[MAX_SERVOS_NR],
    t,h;
    for(j=0;(time[j]<T)&&(j!=npoints-1);j++)
        ;
    // printf("\n %f  %d ",T,j);
    h=time[j]-time[j-1];
    t= (T - time[j-1]) / h;
    for (i=0;i<6;i++)
    {
        dy[i]= yi[j][i]-yi[j-1][i];
        A[i]= -2 * dy[i]/h + (yiprim[j-1][i] + yiprim[j][i]);
        B[i]= -A[i] + dy[i]/h - yiprim[j-1][i];
        y[i]= yi[j-1][i] + (T - time[j-1]) * (yiprim[j-1][i] + t * (B[i] + t * A[i]));
        // y[i]=A[i]*t*t*t + B[i]*t*t + yiprim[j-1][i]*t + yi[j-1][i];
        // v[i]= (3*A[i]*t*t + 2*B[i]*t + yiprim[j-1][i]);
        // a[i]= MAX_SERVOS_NR*A[i]*t + 2*B[i];
    }
}
; // end: bool irp6ot_hermite_spline_generator::calc_hermit()


// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------
bool hermite_spline::first_step (  )
{




    get_pose (starting_pose); // Load the pose specification from the first pose loaded from the trajectory file to the structure 'starting_pose'
    first_time = true;
    last_time = false;

    // Request the coordinates of the current pose of the robot
    switch ( starting_pose.arm_type )
    {
    case MOTOR:
        // printf("\n first step MOTOR");
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = MOTOR;
        break;
    case JOINT:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = JOINT;
        break;
    case XYZ_EULER_ZYZ:
        // printf("\n  first step, XYZ_EULER_ZYZ");
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
        break;
    case XYZ_ANGLE_AXIS:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
        break;
    default:
        throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    } // end: switch

    return true;


}
; // end: bool irp6ot_hermite_spline_generator::first_step ( )

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool hermite_spline::next_step (  )
{


    int i;
    if (last_time)
    {
        fclose(log_file);
        return false;
    }
    if ( first_time )
    {



        // Copy the coordinates of the current robot pose from the virtual image of the robot to the structure 'starting_pose'
        switch ( starting_pose.arm_type )
        {
        case MOTOR:
            for(i=0; i<MAX_SERVOS_NR; i++)
                starting_pose.coordinates[i]=the_robot->EDP_data.current_motor_arm_coordinates[i];
            // for(i=0; i<6; i++) printf("\n next step MOTOR %f",starting_pose.coordinates[i]);
            break;
        case JOINT:
                for(i=0; i<MAX_SERVOS_NR; i++)
                    starting_pose.coordinates[i]=the_robot->EDP_data.current_joint_arm_coordinates[i];
            break;
        case XYZ_EULER_ZYZ:
                for(i=0; i<6; i++)
                    starting_pose.coordinates[i]=the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
            starting_pose.coordinates[6]=the_robot->EDP_data.current_gripper_coordinate;
            // for(i=0; i<6; i++) printf("\n dddddd  %f     %f",starting_pose.coordinates[i],the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i] );
            break;
        case XYZ_ANGLE_AXIS:
                for(i=0; i<6; i++)
                    starting_pose.coordinates[i]=the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
            starting_pose.coordinates[6]=the_robot->EDP_data.current_gripper_coordinate;
            break;
        default:
                throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
        } // end: switch

        fill_hermite_arrays(); // fill arrays 'time', 'yi', 'yiprim'
        the_robot->EDP_data.instruction_type = SET;
        the_robot->EDP_data.set_arm_type = starting_pose.arm_type;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.motion_type = ABSOLUTE;
         the_robot->EDP_data.next_interpolation_type = MIM;
        the_robot->EDP_data.motion_steps = (WORD) (10);
        the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-2;
        first_time = false;
        T = time[0];
        log_file=fopen("log_file.txt","w");
    }

    T=T+TSTEP; // incrementing T

    if (T >= time[npoints-1])
    {
        T=time[npoints-1];
        last_time=true;
    }
    //
    calc_hermit();

    // fprintf(log_file,"%f %f %f %f %f %f %f\n",T,y[0],y[1],y[2],y[3],y[4],y[5]);
    printf("\n\n%f %f %f %f %f %f %f\n",T,y[0],y[1],y[2],y[3],y[4],y[5]);

    // Copy the newly calculated coordinates of the next robot pose from the array y to the virtual image of the robot
    switch ( starting_pose.arm_type )
    {
    case MOTOR:
        for (i = 0; i < MAX_SERVOS_NR; i++)
            the_robot->EDP_data.next_motor_arm_coordinates[i] = y[i];
        break;
    case JOINT:
            for (i = 0; i < MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_joint_arm_coordinates[i] = y[i];
        break;
    case XYZ_EULER_ZYZ:
            for (i = 0; i < 6; i++)
                the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] =y[i];
        the_robot->EDP_data.next_gripper_coordinate = y[6];
        break;
    case XYZ_ANGLE_AXIS:
            for (i = 0; i < 6; i++)
                the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = y[i];
        the_robot->EDP_data.next_gripper_coordinate = y[6];
        break;
    default:
            throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    } // end: switch

    return true;
} // end: bool irp6ot_hermite_spline_generator::next_step ( )


// ########################################################################################################
// ########################################################################################################
// ############## Definnitions for methods of irp6ot_natural_spline_generator #############################
// ########################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------irp6ot_natural_spline_generator - constuctor ---------------------------
// ----------------------------------------------------------------------------------------------
natural_spline::natural_spline(common::task::ecp_task& _ecp_task,
        double interval, double ts):
ecp_teach_in_generator (_ecp_task)
{
    TSTEP=interval;
    INTERVAL=interval;
    TS=ts;
    int i,j;

    for (i=0;i<MAX_SERVOS_NR;i++)
    {					// setting the values of the default velocities
        v_def_motor[i]=90;
        v_def_joint[i]=50;
        v_def_xyz_angles[i]=50;
        for (j=0;j<NPOINTS;j++)
        {
            yibis[j][i]=0;
        }
    }
    v_def_xyz_euler[0]=0.05;
    v_def_xyz_euler[1]=0.03;
    v_def_xyz_euler[2]=0.05;
    v_def_xyz_euler[3]=0.3;
    v_def_xyz_euler[4]=20;
    v_def_xyz_euler[5]=20;
    v_def_xyz_euler[6]=20;
    v_def_xyz_euler[7]=20;
}
;// end: irp6ot_natural_spline_generator::irp6ot_natural_spline_generator()


// ----------------------------------------------------------------------------------------------
// ----------------------  metoda fill_natural_arrays ----------------------------
// ----------------------------------------------------------------------------------------------
void natural_spline::fill_natural_arrays(void)
{
    if (pose_list.empty())
        throw ECP_error (NON_FATAL_ERROR, NON_COMPATIBLE_LISTS);
    int i,j;

    double dx1[MAX_SERVOS_NR],dx2[MAX_SERVOS_NR],dy1[MAX_SERVOS_NR],dy2[MAX_SERVOS_NR],h,p[MAX_SERVOS_NR],dtime[MAX_SERVOS_NR];
    double aa[NPOINTS][MAX_SERVOS_NR],c[NPOINTS][MAX_SERVOS_NR],d[NPOINTS][MAX_SERVOS_NR],
    q[NPOINTS][MAX_SERVOS_NR],u[NPOINTS][MAX_SERVOS_NR];  // aux variables
    npoints = pose_list_length() + 1;   // number of control poses - the one additional for the first pose
    if(npoints<3)
        throw ECP_error (NON_FATAL_ERROR, NON_COMPATIBLE_LISTS);

    // Copy the values of the coordinates of the current robot pose from the virtual image of the robot to the first entry in the array yi
    for (i=0;i<MAX_SERVOS_NR;i++)
        yi[0][i]=starting_pose.coordinates[i];
    initiate_pose_list();  // reset list pointers
    j=1;
    while (pose_list_iterator != pose_list.end()) // Copy the values of the coordinates of the control poses from the list pose_list_ptr to the array yi
    {
        for (i=0;i<MAX_SERVOS_NR;i++)
            yi[j][i]=pose_list_iterator->coordinates[i];
        // printf("\n fill arrays : yi[%d][2]  %f\n",i,yi[i][2]);
        pose_list_iterator++;
        j++;
    }

    // ################### calculating times between control poses  - filling 'time'
    time[0]=0;
    for (j=1;j<=npoints;j++)
    {
        time[j]=time[j-1];
        switch ( starting_pose.arm_type )
        {
        case MOTOR:
            for(i=0;i<MAX_SERVOS_NR;i++)
            {
                dtime[i]=TS*(ceil((fabs(yi[j][i]-yi[j-1][i])/v_def_motor[i])/INTERVAL)*INTERVAL);
                if(dtime[i]<TS*0.1)
                    dtime[i]=TS*0.1;
                if (time[j]<time[j-1]+dtime[i])
                    time[j]=time[j-1]+dtime[i];
                // printf("\n   %f",dtime[i]);
            }
            break;
        case JOINT:
            for(i=0;i<MAX_SERVOS_NR;i++)
            {
                dtime[i]=TS*(ceil((fabs(yi[j][i]-yi[j-1][i])/v_def_joint[i])/INTERVAL)*INTERVAL);
                if(dtime[i]<TS*0.1)
                    dtime[i]=TS*0.1;
                if (time[j]<time[j-1]+dtime[i])
                    time[j]=time[j-1]+dtime[i];
                // printf("\n   %f",dtime[i]);
            }
            break;
        case XYZ_EULER_ZYZ:
            for(i=0;i<7;i++)
            {
                dtime[i]=TS*(ceil((fabs(yi[j][i]-yi[j-1][i])/v_def_xyz_euler[i])/INTERVAL)*INTERVAL);
                if(dtime[i]<TS*0.1)
                    dtime[i]=TS*0.1;
                if (time[j]<time[j-1]+dtime[i])
                    time[j]=time[j-1]+dtime[i];
                // printf("\n   %f",dtime[i]);
            }
            break;
        case XYZ_ANGLE_AXIS:
            for(i=0;i<7;i++)
            {
                dtime[i]=TS*(ceil((fabs(yi[j][i]-yi[j-1][i])/v_def_xyz_angles[i])/INTERVAL)*INTERVAL);
                if(dtime[i]<TS*0.1)
                    dtime[i]=TS*0.1;
                if(time[j]<time[j-1]+dtime[i])
                    time[j]=time[j-1]+dtime[i];
                // printf("\n   %f",dtime[i]);
            }
            break;
        default:
            throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
        } // end: switch   */
        // printf("\ntime[%d] %f", j,time[j]);
    }
    // #####################

    // filling "yibis"
    for (i=0;i<MAX_SERVOS_NR;i++)
    {
        for (j=1;j<=(npoints-2);j++)
        {
            dx1[i]=time[j+1]-time[j];
            dx2[i]=time[j]-time[j-1];
            dy1[i]=yi[j+1][i]-yi[j][i];
            dy2[i]=yi[j][i]-yi[j-1][i];
            p[i]=time[j+1]-time[j-1];
            c[j][i]=dx1[i]/p[i];
            aa[j][i]=1-c[j][i];
            d[j][i]=MAX_SERVOS_NR*(dy1[i]/dx1[i]-dy2[i]/dx2[i])/p[i];
        }

        h=1/(time[1]-time[0]);
        p[i]=1/(1-0.25*aa[1][i]);
        dx1[i]=3*h*((yi[1][i]-yi[0][i])*h-0);
        c[1][i]=c[1][i]*p[i];
        d[1][i]=(d[1][i]-aa[1][i]*dx1[i])*p[i];

        h=1/(time[npoints-1]-time[npoints-2]);
        p[i]=1/(1-0.25*c[npoints-2][i]);
        dx2[i]=3*h*(0-(yi[npoints-1][i]-yi[npoints-2][i])*h);
        aa[npoints-2][i]=aa[npoints-2][i]*p[i];
        d[npoints-2][i]=(d[npoints-2][i]-c[npoints-2][i]*dx2[i])*p[i];


        q[1][i]=-0.5*c[1][i];
        u[1][i]=0.5*d[1][i];
        for (j=2;j<=(npoints-2);j++)
        {
            p[i]=1/(2+aa[j][i]*q[j-1][i]);
            q[j][i]=-c[j][i]*p[i];
            u[j][i]=(d[j][i]-aa[j][i]*u[j-1][i])*p[i];
        }

        yibis[npoints-2][i]=u[npoints-2][i];

        for (j=(npoints-3);j>=1;j--)
        {
            yibis[j][i]=yibis[j+1][i]*q[j][i]+u[j][i];
        }

        yibis[0][i]=dx1[i]-0.5*yibis[1][i];
        yibis[npoints-1][i]=dx2[i]-0.5*yibis[npoints-2][i];
    }

    // sr_ecp_msg->message("fill_natural_arrays ");
}
;// end: irp6ot_natural_spline_generator::fill_natural_arrays()


// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	calc_natural -------------------------------------------------
// ----------------------------------------------------------------------------------------------

void natural_spline::calc_natural()
{
    int i,j;
    double A[MAX_SERVOS_NR],B[MAX_SERVOS_NR],h,h1,h2;
    for (i=0;i<MAX_SERVOS_NR;i++)
    {
        for(j=0;(time[j]<T)&&(j!=npoints-1);j++)
            ;

        h1=T-time[j-1];
        h2=time[j]-T;
        h=time[j]-time[j-1];
        // a[i]=(yibis[j][i]*h1+yibis[j-1][i]*h2)/h;

        A[i]=(yi[j][i]-yi[j-1][i])/h-h*(yibis[j][i]-yibis[j-1][i])/6;
        B[i]=yi[j-1][i]-h*h*yibis[j-1][i]/6;
        // v[i]=0.5*(yibis[j][i]*h1*h1-yibis[j-1][i]*h2*h2)/h+A[i];
        y[i]=(yibis[j][i]*h1*h1*h1+yibis[j-1][i]*h2*h2*h2)/h/6+A[i]*h1+B[i];
    }

}
; // end: rp6ot_natural_spline_generator::calc_natural()
// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool natural_spline::first_step ()
{




    get_pose (starting_pose);  // Load the pose specification from the first pose loaded from the trajectory file to the structure 'starting_pose'
    first_time = true;
    last_time=false;

    // Request the coordinates of the current pose of the robot
    switch ( starting_pose.arm_type )
    {
    case MOTOR:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = MOTOR;
        break;
    case JOINT:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = JOINT;
        break;
    case XYZ_EULER_ZYZ:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
        break;
    case XYZ_ANGLE_AXIS:
        the_robot->EDP_data.instruction_type = GET;
        the_robot->EDP_data.get_type = ARM_DV; // ARM
        the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
        break;
    default:
        throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    } // end: switch

    // sr_ecp_msg->message("natural - first step");
    return true;


}
; // end: irp6ot_natural_spline_generator::first_step()

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------
bool natural_spline::next_step ()
{


    int i;
    if (last_time)
    {
        fclose(log_file);
        return false;
    }
    if ( first_time )
    {


        // Copy the coordinates of the current robot pose from the virtual image of the robot to the structure 'starting_pose'
        switch ( starting_pose.arm_type )
        {
        case MOTOR:
            for(i=0; i<MAX_SERVOS_NR; i++)
                starting_pose.coordinates[i]=the_robot->EDP_data.current_motor_arm_coordinates[i];
            // for(i=0; i<6; i++)printf("\n next step MOTOR %f",starting_pose.coordinates[i]);
            break;
        case JOINT:
                for(i=0; i<MAX_SERVOS_NR; i++)
                    starting_pose.coordinates[i]=the_robot->EDP_data.current_joint_arm_coordinates[i];
            break;
        case XYZ_EULER_ZYZ:
                for(i=0; i<6; i++)
                    starting_pose.coordinates[i]=the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
            starting_pose.coordinates[6]=the_robot->EDP_data.current_gripper_coordinate;
            // for(i=0; i<6; i++) printf("\n dddddd  %f     %f",starting_pose.coordinates[i],the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i] );
            break;
        case XYZ_ANGLE_AXIS:
                for(i=0; i<6; i++)
                    starting_pose.coordinates[i]=the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
            starting_pose.coordinates[6]=the_robot->EDP_data.current_gripper_coordinate;
            break;
        default:
                throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
        } // end: switch

        fill_natural_arrays();
        the_robot->EDP_data.instruction_type = SET;
        the_robot->EDP_data.set_arm_type = starting_pose.arm_type;
        the_robot->EDP_data.set_type = ARM_DV; // ARM
        the_robot->EDP_data.motion_type = ABSOLUTE;
         the_robot->EDP_data.next_interpolation_type = MIM;
        the_robot->EDP_data.motion_steps = (WORD) (10);
        the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-2;
        first_time = false;
        T = time[0];
        log_file=fopen("log_file.txt","w");
        // sr_ecp_msg->message("next step - first time ");
    }

    T=T+TSTEP;
    if (T >= time[npoints-1])
    {
        T=time[npoints-1];
        last_time=true;
    }

    calc_natural();

    // fprintf(log_file,"%f %f %f %f %f %f %f\n",T,y[0],y[1],y[2],y[3],y[4],y[5]);
    printf("\n\n%f %f %f %f %f %f %f\n",T,y[0],y[1],y[2],y[3],y[4],y[5]);

    // Copy the newly calculated coordinates of the next robot pose from the array y to the virtual image of the robot
    switch ( starting_pose.arm_type )
    {
    case MOTOR:
        for (i = 0; i < MAX_SERVOS_NR; i++)
            the_robot->EDP_data.next_motor_arm_coordinates[i] = y[i];
        break;
    case JOINT:
            for (i = 0; i < MAX_SERVOS_NR; i++)
                the_robot->EDP_data.next_joint_arm_coordinates[i] = y[i];
        break;
    case XYZ_EULER_ZYZ:
            for (i = 0; i < 6; i++)
                the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] =y[i];
        the_robot->EDP_data.next_gripper_coordinate = y[6];
        break;
    case XYZ_ANGLE_AXIS:
            for (i = 0; i < 6; i++)
                the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = y[i];
        the_robot->EDP_data.next_gripper_coordinate = y[6];
        break;
    default:
            throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    } // end: switch
    return true;
}
; // end: irp6ot_natural_spline_generator::next_step()

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
