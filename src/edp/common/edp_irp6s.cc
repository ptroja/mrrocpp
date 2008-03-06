// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6s.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody wspolne dla robotow IRp-6
// 				- definicja metod klasy edp_irp6s_effector
//
// Autor:		tkornuta
// Data:		14.02.2007
// -------------------------------------------------------------------------

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <process.h>
#include <sys/netmgr.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mis_fun.h"
#include "edp/common/edp.h"
#include "lib/mathtr.h"



/*--------------------------------------------------------------------------*/
edp_irp6s_effector::edp_irp6s_effector (configurator &_config, ROBOT_ENUM l_robot_name) :
        edp_irp6s_and_conv_effector (_config, l_robot_name)
{
}
;

void edp_irp6s_effector::initialize (void)
{}
;


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::compute_xyz_euler_zyz (const c_buffer &instruction)
{
    // obliczenia dla ruchu ramienia (kocwk: XYZ_EULER_ZYZ)
    /* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
    /* Zlecenie transformerowi przeliczenie wspolrzednych */

    const double* p;   // wskanik miejsca w strukturze przesanej z ECP, w ktorym znajduj sie wspolrzedne
    if (instruction.is_set_arm())
    {
        // przyslano dane dotyczace koncowki
        motion_type = instruction.motion_type;
        motion_steps = instruction.motion_steps;
        value_in_step_no = instruction.value_in_step_no;
        p = (double*) instruction.arm.coordinate_def.arm_coordinates;
    }
    for (int i=0;i<6;i++)
        rb_obj->step_data.current_kartez_position[i] = instruction.arm.coordinate_def.arm_coordinates[i];

    // dla robotow track i postument - oblicz chwytak
    if ((robot_name == ROBOT_IRP6_ON_TRACK) || (robot_name == ROBOT_IRP6_POSTUMENT))
    {
        desired_joints_tmp[gripper_servo_nr] = instruction.arm.coordinate_def.gripper_coordinate;
    }

    // if ( (value_in_step_no <= 0) || (motion_steps <= 0) || (value_in_step_no   > motion_steps + 1) )
    if ( (motion_steps <= 0) || (value_in_step_no   > motion_steps + 1) )// by Y
        throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
    switch (motion_type)
    {
    case ABSOLUTE:   // ruch bezwzgledny
        arm_abs_xyz_eul_zyz_2_frame(p);
        break;
    case RELATIVE:   // ruch wzgledny
        arm_rel_xyz_eul_zyz_2_frame(p);
        break;
    default:
        throw NonFatal_error_2(INVALID_MOTION_TYPE);
    }

    // Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
    get_current_kinematic_model()->e2i_transform(desired_joints_tmp, current_joints, &desired_end_effector_frame);
    // Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
    get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);

    // kinematyka nie stwierdzila bledow, przepisanie wartosci
    for (int i=0; i< number_of_servos; i++)
    {
        desired_joints[i] = desired_joints_tmp[i];
        desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
    }

}
/*------------------------------------------------------------------*/




/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::compute_xyz_angle_axis (const c_buffer &instruction)
{
    // obliczenia dla ruchu ramienia (kocwk: XYZ_ANGLE_AXIS)
    /* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
    /* Zlecenie transformerowi przeliczenie wspolrzednych */
    const double* p;   // wskanik miejsca w strukturze przesanej z ECP, w ktorym znajduj sie wspolrzedne
    if ( instruction.is_set_arm() || instruction.is_set_rmodel() )
    {
        // przyslano dane dotyczace narzedzia i koncowki
        motion_type = instruction.motion_type;
        motion_steps = instruction.motion_steps;
        value_in_step_no = instruction.value_in_step_no;
        p = &instruction.arm.coordinate_def.arm_coordinates[0];
    }

    // dla robotow track i postument - oblicz chwytak
    if ((robot_name == ROBOT_IRP6_ON_TRACK) || (robot_name == ROBOT_IRP6_POSTUMENT))
    {
        desired_joints_tmp[gripper_servo_nr] = instruction.arm.coordinate_def.gripper_coordinate;
    }

    // if ( (value_in_step_no <= 0) || (motion_steps <= 0) || (value_in_step_no   > motion_steps + 1) )
    if ( (motion_steps <= 0) || (value_in_step_no   > motion_steps + 1) )// by Y
        throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
    switch (motion_type)
    {
    case ABSOLUTE:   // ruch bezwzgledny
        arm_abs_xyz_aa_2_frame(p);
        break;
    case RELATIVE:   // ruch wzgledny
        arm_rel_xyz_aa_2_frame(p);
        break;
    default:
        throw NonFatal_error_2(INVALID_MOTION_TYPE);
    }
    
    // Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
    get_current_kinematic_model()->e2i_transform(desired_joints_tmp, current_joints, &desired_end_effector_frame);
    // Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
    get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);
    // kinematyka nie stwierdzila bledow, przepisanie wartosci
    for (int i=0; i< number_of_servos; i++)
    {
        desired_joints[i] = desired_joints_tmp[i];
        desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
    }
}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::compute_frame (const c_buffer &instruction)
{
    // obliczenia dla ruchu ramienia (kocwk: FRAME)
    /* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
    /* Zlecenie transformerowi przeliczenie wspolrzednych */
    frame_tab p_m;   // wskanik miejsca w strukturze przesanej z ECP, w ktorym znajduj sie wspolrzedne
    if ( instruction.is_set_rmodel() || instruction.is_set_arm() )
    {
        // przyslano dane dotyczace narzedzia lub koncowki
        motion_type = instruction.motion_type;
        motion_steps = instruction.motion_steps;
        value_in_step_no = instruction.value_in_step_no;

        copy_frame(p_m, instruction.arm.frame_def.arm_frame);

    } // end: then

    // dla robotow track i postument - oblicz chwytak
    if ((robot_name == ROBOT_IRP6_ON_TRACK) || (robot_name == ROBOT_IRP6_POSTUMENT))
    {
        desired_joints_tmp[gripper_servo_nr] = instruction.arm.frame_def.gripper_coordinate;
    }

    if ( (value_in_step_no <= 0) || (motion_steps <= 0) || (value_in_step_no   > motion_steps + 1) )
    {
        throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
    }
    switch (motion_type)
    {
    case ABSOLUTE:   // ruch bezwzgledny
        arm_abs_frame_2_frame(p_m);
        break;
    case RELATIVE:   // ruch wzgledny
        arm_rel_frame_2_frame(p_m);
        break;
    default:
        throw NonFatal_error_2(INVALID_MOTION_TYPE);
    } // end: switch (instruction.motion_type)
    // Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
    get_current_kinematic_model()->e2i_transform(desired_joints_tmp, current_joints, &desired_end_effector_frame);
    // Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
    get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);

    // kinematyka nie stwierdzila bledow, przepisanie wartosci
    for (int i=0; i< number_of_servos; i++)
    {
        desired_joints[i] = desired_joints_tmp[i];
        desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
    }
}
/*--------------------------------------------------------------------------*/

// Przeksztacenie definicji narzedzia z postaci
// TOOL_FRAME do postaci TOOL_XYZ_ANGLE_AXIS oraz przepisanie wyniku
// przeksztacenia do wewntrznych struktur danych REPLY_BUFFER.
void edp_irp6s_effector::tool_frame_2_xyz_aa (void)
{
    reply.rmodel_type = TOOL_XYZ_ANGLE_AXIS;
    switch (reply.reply_type)
    {
    case RMODEL:
    case RMODEL_INPUTS:
    case ARM_RMODEL:
    case ARM_RMODEL_INPUTS:
        get_current_kinematic_model()->tool.get_xyz_angle_axis(reply.rmodel.tool_coordinate_def.tool_coordinates);
        break;
    default:
        // Blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia.
        throw NonFatal_error_2(ERROR_IN_RMODEL_REQUEST);
    } // end: switch (reply_type)
}
; // end: edp_irp6s_effector::tool_frame_2_xyz_aa


// Przeksztacenie definicji narzedzia z postaci
// TOOL_FRAME do postaci TOOL_XYZ_EULER_ZYZ oraz przepisanie wyniku
// przeksztacenia do wewntrznych struktur danych REPLY_BUFFER.
void edp_irp6s_effector::tool_frame_2_xyz_eul_zyz (void)
{
    reply.rmodel_type = TOOL_XYZ_EULER_ZYZ;
    switch (reply.reply_type)
    {
    case RMODEL:
    case RMODEL_INPUTS:
    case ARM_RMODEL:
    case ARM_RMODEL_INPUTS:
        get_current_kinematic_model()->tool.get_xyz_euler_zyz(reply.rmodel.tool_coordinate_def.tool_coordinates);
        break;
    default:
        // Blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia.
        throw NonFatal_error_2(ERROR_IN_RMODEL_REQUEST);
    } // end: switch (reply_type)
}
; // end: edp_irp6s_effector::tool_frame_2_xyz_eul_zyz


// Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
// z wewntrznych struktur danych TRANSFORMATORa
// do wewntrznych struktur danych REPLY_BUFFER
void edp_irp6s_effector::tool_frame_2_frame_rep (void)
{
    reply.rmodel_type = TOOL_FRAME;
    switch (reply.reply_type)
    {
    case RMODEL:
    case RMODEL_INPUTS:
    case ARM_RMODEL:
    case ARM_RMODEL_INPUTS:
        get_current_kinematic_model()->tool.get_frame_tab(reply.rmodel.tool_frame_def.tool_frame);
        break;
    default:
        // Blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia.
        throw NonFatal_error_2(ERROR_IN_RMODEL_REQUEST);
    } // end: switch (reply_type)
}
; // end: edp_irp6s_effector::tool_frame_2_frame_rep


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::tool_axially_symmetrical_frame_2_xyz_eul_zy (void)
{
    // Przeksztacenie definicji narzdzia z postaci
    // TOOL_FRAME do postaci TOOL_AS_XYZ_EULER_ZY oraz przepisanie wyniku
    // przeksztacenia do wewntrznych struktur danych
    // REPLY_BUFFER

    double psi, fi;
    double v6[3],q6[3];

    double xyz_aa[6];

    Homog_matrix A(get_current_kinematic_model()->tool);
    A.get_xyz_angle_axis(xyz_aa);

    q6[0] =xyz_aa[0];
    q6[1] = xyz_aa[1];
    q6[2] = xyz_aa[2];

    v6[0] = xyz_aa[3];
    v6[1] = xyz_aa[4];
    v6[2] = xyz_aa[5];

#define EPS_V 1.0e-6

    if ( fabs(v6[0]) > EPS_V)
    {
        fi   = atan2(v6[1],v6[0]);
        psi = atan2(-v6[2], sqrt(v6[0]*v6[0]+v6[1]*v6[1]));
    }
    else
        if (fabs(v6[1]) >EPS_V)
        {
            psi = atan2(-v6[2], fabs(v6[1]));
            if (v6[1] > 0)
                fi   =   M_PI/2;
            else
                fi   = -M_PI/2;
        }
        else
        {
            fi   = 0;   /* Dowolna wartož k†ta fi, poniewa§ Vx = Vy = 0 */
            if (v6[2] > 0)
                psi = -M_PI/2;
            else
                psi =   M_PI/2;
        }

    reply.rmodel_type = TOOL_AS_XYZ_EULER_ZY;

    reply.rmodel.tool_coordinate_def.tool_coordinates[0]=q6[0];
    reply.rmodel.tool_coordinate_def.tool_coordinates[1]=q6[1];
    reply.rmodel.tool_coordinate_def.tool_coordinates[2]=q6[2];
    reply.rmodel.tool_coordinate_def.tool_coordinates[3]= fi;
    reply.rmodel.tool_coordinate_def.tool_coordinates[4]= psi;
    reply.rmodel.tool_coordinate_def.tool_coordinates[5]= 0;

}
; // end: edp_irp6s_effector::tool_axially_symmetrical_frame_2_xyz_eul_zyz
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::tool_xyz_eul_zyz_2_frame (c_buffer *instruction)
{
    // Przeksztacenie definicji narzedzia z postaci
    // TOOL_XYZ_EULER_ZYZ do postaci TOOL_FRAME oraz przepisanie wyniku
    // przeksztacenia do wewntrznych struktur danych
    // TRANSFORMATORa
    // by Y UWAGA NIEPRZETESTOWANE

    double x, y, z;					// wspolrzedne wektora przesuniecia
    double alfa, beta, gamma;	// Katy Eulera

    // przepisanie z tablicy pakietu komunikacyjnego
    x = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[0];
    y = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[1];
    z = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[2];

    alfa = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[3];
    beta = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[4];
    gamma = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[5];

    Homog_matrix A_B_T (Homog_matrix::MTR_XYZ_EULER_ZYZ, x, y, z, alfa, beta, gamma);
    // Sprawdzenie, czy macierz jest jednorodna.
    if (!(A_B_T.is_valid()))
        throw NonFatal_error_2(INVALID_HOMOGENEOUS_MATRIX);
    // Ustawienie macierzy reprezentujacej narzedzie.
    get_current_kinematic_model()->tool = A_B_T;

}
; // end: edp_irp6s_effector::tool_xyz_eul_zyz_2_frame
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::tool_frame_2_frame (c_buffer *instruction)
{
    // Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
    // do wewntrznych struktur danych TRANSFORMATORa
    // Sprawdzenie czy przepisana macierz jest jednorodna
    // Jezeli nie, to wyzwalany jest wyjatek.

    if ( instruction->is_set_rmodel() || instruction->is_set_arm())
    {
        // Przyslano dane dotyczace narzedzia i koncowki.
        Homog_matrix A_B_T ((*instruction).rmodel.tool_frame_def.tool_frame);
        // Sprawdzenie poprawnosci macierzy
        if (!(A_B_T.is_valid()))
            throw NonFatal_error_2(INVALID_HOMOGENEOUS_MATRIX);
        // Przepisanie do kinematyki.
        get_current_kinematic_model()->tool = A_B_T;
    }
    ;//: if
}
; // end: edp_irp6s_effector::tool_frame_2_frame
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_abs_xyz_aa_2_frame (const double *p)
{
    double alfa;				// kat obrotu
    double x, y, z;			// wspolrzedne wektora przesuniecia
    double kx, ky, kz;		// specyfikacja wektora, wokol ktorego obracany jest uklad

    // przepisanie z tablicy pakietu komunikacyjnego
    x = p[0];
    y = p[1];
    z = p[2];

    // przepisane wartosci pomnozone sa przez kat alfa
    kx = p[3];
    ky = p[4];
    kz = p[5];

    // obliczenie kata obrotu alfa i wartosci funkcji trygonometrycznych
    alfa = sqrt(kx*kx + ky*ky + kz*kz);

    // korekta wartosci x, y, z
    if((alfa   < ALFA_SENSITIVITY) && (alfa > -ALFA_SENSITIVITY))
    {
        Homog_matrix A_B_T(x, y, z);
        A_B_T.get_frame_tab(desired_end_effector_frame); 		// przepisanie uzyskanego wyniku do transformera
    }
    else
    {
        kx = kx/alfa;
        ky = ky/alfa;
        kz = kz/alfa;
        Homog_matrix A_B_T(kx, ky, kz, alfa, x, y, z);
        A_B_T.get_frame_tab(desired_end_effector_frame); 		// przepisanie uzyskanego wyniku do transformera
    }
}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_abs_xyz_eul_zyz_2_frame (const double *p)
{}


/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_abs_frame_2_frame (frame_tab p_m)
{
    // Przepisanie definicji koncowki danej
    // w postaci TRANS wyraonej bezwzgldnie
    // do wewntrznych struktur danych TRANSFORMATORa
    copy_frame(desired_end_effector_frame, p_m);


}
; // end: edp_irp6s_effector::arm_abs_frame_2_frame
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_rel_xyz_aa_2_frame (const double* p)
{

    double alfa;			// kat obrotu

    double x, y, z;			// wspolrzedne wektora przesuniecia
    double kx, ky, kz;		// specyfikacja wektora, wokol ktorego obracany jest uklad

    Homog_matrix G_R_T;

    // pobranie aktualnej macierzy przeksztalcenia
    Homog_matrix G_K_T(current_end_effector_frame);

    // przepisanie z tablicy pakietu komunikacyjnego
    x = p[0];
    y = p[1];
    z = p[2];

    // przepisane wartosci pomnozone sa przez kat alfa
    kx = p[3];
    ky = p[4];
    kz = p[5];

    // obliczenie kata obrotu alfa i wartosci funkcji trygonometrycznych
    alfa = sqrt(kx*kx + ky*ky + kz*kz);
    // korekta wartosci x, y, z
    if((alfa   < ALFA_SENSITIVITY) && (alfa > -ALFA_SENSITIVITY))
    {
        Homog_matrix K_R_T(x, y, z);
        G_R_T = G_K_T * K_R_T;			// obliczenie macierzy przeksztalcenia
    }
    else
    {
        kx /=alfa;
        ky /=alfa;
        kz /=alfa;
        Homog_matrix K_R_T(kx, ky, kz, alfa, x, y, z);
        G_R_T = G_K_T * K_R_T;			// obliczenie macierzy przeksztalcenia
    }
    G_R_T.get_frame_tab(desired_end_effector_frame);

}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_rel_xyz_eul_zyz_2_frame (const double* p)
{

    double x, y, z;			// wspolrzedne wektora przesuniecia
    double alfa, beta, gamma;	// Katy Eulera

    // przepisanie z tablicy pakietu komunikacyjnego
    x = p[0];
    y = p[1];
    z = p[2];
    alfa = p[3];
    beta = p[4];
    gamma = p[5];
    Homog_matrix K_R_T (Homog_matrix::MTR_XYZ_EULER_ZYZ, x, y, z, alfa, beta, gamma);
    Homog_matrix G_K_T(current_end_effector_frame);	// pobranie aktualnej macierzy przeksztalcenia
    Homog_matrix G_R_T = G_K_T * K_R_T;
    G_R_T.get_frame_tab(desired_end_effector_frame);			// przepisanie uzyskanego wyniku do transformera

}
; // end: edp_irp6s_effector::arm_rel_xyz_eul_zyz_2_frame
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_rel_frame_2_frame (frame_tab p_m)
{
    // Przepisanie definicji koncowki danej
    // w postaci TRANS wyraonej wzgldnie
    // do wewntrznych struktur danych TRANSFORMATORa
    // Przepisanie z przemnozeniem
    Homog_matrix A_B_T_arg(p_m);
    Homog_matrix A_B_T_des (desired_end_effector_frame);
    A_B_T_des *= A_B_T_arg;
    A_B_T_des.get_frame_tab(desired_end_effector_frame);

    // matrix_mult(desired_end_effector_frame, *p);
    // sprawdzi przekroczenie dopuszczalnego zakresu oraz poprawno macierzy jednorodnej
}
; // end: edp_irp6s_effector::arm_rel_frame_2_frame
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_frame_2_xyz_aa (void)
{

    Homog_matrix A(current_end_effector_frame);
    reply.arm_type = XYZ_ANGLE_AXIS;
    switch (reply.reply_type)
    {
    case ARM:
    case ARM_INPUTS:
    case ARM_RMODEL:
    case ARM_RMODEL_INPUTS:
        A.get_xyz_angle_axis(reply.arm.coordinate_def.arm_coordinates);
        break;
        // case FORCE:

        // for Y
        // miejsce do uzupelnienia
        // 	break;
    default:
        throw NonFatal_error_2(STRANGE_GET_ARM_REQUEST);
    }

    // dla robotow track i postument - oblicz chwytak
    if ((robot_name == ROBOT_IRP6_ON_TRACK) || (robot_name == ROBOT_IRP6_POSTUMENT))
    {
        reply.arm.coordinate_def.gripper_reg_state = servo_gripper_reg_state;
        reply.arm.coordinate_def.gripper_coordinate = current_joints[gripper_servo_nr];
    }

}
; // end: edp_irp6s_effector::arm_frame_2_xyz_aa
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_frame_2_xyz_eul_zyz ()
{}
;

/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::tool_xyz_aa_2_frame (c_buffer *instruction)
{
    // Przeksztacenie definicji narzedzia z postaci
    // TOOL_XYZ_ANGLE_AXIS do postaci TOOL_FRAME oraz przepisanie wyniku
    // przeksztacenia do wewntrznych struktur danych
    // TRANSFORMATORa
    // by Y UWAGA NIEPRZETESTOWANE


    double alfa;				// kat obrotu
    double x, y, z;			// wspolrzedne wektora przesuniecia
    double kx, ky, kz;		// specyfikacja wektora, wokol ktorego obracany jest uklad

    // przepisanie z tablicy pakietu komunikacyjnego
    x = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[0];
    y = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[1];
    z = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[2];

    // przepisane wartosci pomnozone sa przez kat alfa
    kx = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[3];
    ky = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[4];
    kz = (*instruction).rmodel.tool_coordinate_def.tool_coordinates[5];

    // obliczenie kata obrotu alfa i wartosci funkcji trygonometrycznych
    alfa = sqrt(kx*kx + ky*ky + kz*kz);


    // korekta wartosci x, y, z
    if((alfa   < ALFA_SENSITIVITY) && (alfa > -ALFA_SENSITIVITY))
    {
        Homog_matrix A_B_T(x, y, z);
        // Sprawdzenie, czy macierz jest jednorodna.
        if (!(A_B_T.is_valid()))
            throw NonFatal_error_2(INVALID_HOMOGENEOUS_MATRIX);
        // Ustawienie macierzy reprezentujacej narzedzie.
        get_current_kinematic_model()->tool = A_B_T;
    }
    else
    {
        kx = kx/alfa;
        ky = ky/alfa;
        kz = kz/alfa;
        Homog_matrix A_B_T(kx, ky, kz, alfa, x, y, z);
        // Sprawdzenie, czy macierz jest jednorodna.
        if (!(A_B_T.is_valid()))
            throw NonFatal_error_2(INVALID_HOMOGENEOUS_MATRIX);
        // Ustawienie macierzy reprezentujacej narzedzie.
        get_current_kinematic_model()->tool = A_B_T;
    }
}
; // end: edp_irp6s_effector::tool_xyz_aa_2_frame
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::tool_axially_symmetrical_xyz_eul_zy_2_frame (c_buffer *instruction)
{
    // Przeksztacenie definicji narzdzia z postaci
    // TOOL_AS_XYZ_EULER_ZY do postaci TOOL_FRAME oraz przepisanie wyniku
    // przeksztacenia do wewntrznych struktur danych
    // TRANSFORMATORa
    double *t;
    t = &(*instruction).rmodel.tool_coordinate_def.tool_coordinates[0];

    double psi, fi;			//wspolrzedne wektora przesuniecia
    double v6[3];

    psi = t[3];
    fi = t[4];

    //   void Euler_to_Vector( double fi, double psi, double v[3])
    v6[0] = cos(psi)*cos(fi);
    v6[1] = cos(psi)*sin(fi);
    v6[2] = -sin(psi);

    double alfa;			//kat obrotu
    double x, y, z;			//wspolrzedne wektora przesuniecia
    double kx, ky, kz;		//specyfikacja wektora, wokol ktorego obracany jest uklad

    Homog_matrix G_R_T;

    //pobranie aktualnej macierzy przeksztalcenia
    Homog_matrix G_K_T(get_current_kinematic_model()->tool);

    //przepisanie z tablicy pakietu komunikacyjnego
    x = *t;
    y = *(t+1);
    z = *(t+2);

    //przepisane wartosci pomnozone sa przez kat alfa
    kx = v6[0];
    ky = v6[1];
    kz = v6[2];

    //obliczenie kata obrotu alfa i wartosci funkcji trygonometrycznych
    alfa = sqrt(kx*kx + ky*ky + kz*kz);


    //korekta wartosci x, y, z
    if((alfa   < ALFA_SENSITIVITY) && (alfa > -ALFA_SENSITIVITY))
    {

        Homog_matrix K_R_T(x, y, z);

        //obliczenie macierzy przeksztalcenia
        G_R_T = G_K_T * K_R_T;

    }
    else
    {
        kx = kx/alfa;
        ky = ky/alfa;
        kz = kz/alfa;

        Homog_matrix K_R_T(kx, ky, kz, alfa, x, y, z);

        //obliczenie macierzy przeksztalcenia
        G_R_T = G_K_T * K_R_T;
    }

    // Ustawienie macierzy reprezentujacej narzedzie.
    get_current_kinematic_model()->tool = G_R_T;

}
; // end: edp_irp6s_effector::tool_axially_symmetrical_xyz_eul_zy_2_frame
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::arm_frame_2_frame (void)
{
    // Przepisanie definicji koncowki danej w postaci
    // TRANS z wewntrznych struktur danych TRANSFORMATORa
    // do wewntrznych struktur danych REPLY_BUFFER
    reply.arm_type = FRAME;
    switch (reply.reply_type)
    {
    case ARM:
    case ARM_INPUTS:
    case ARM_RMODEL:
    case ARM_RMODEL_INPUTS:
        Homog_matrix::copy_frame_tab(reply.arm.frame_def.arm_frame, current_end_effector_frame);
        for(int i=0; i < 6; i++)
        {
            reply.PWM_value[i] = PWM_value[i];
            reply.current[i] = current[i];
        }
        break;
    default: // blad:
        throw NonFatal_error_2(STRANGE_GET_ARM_REQUEST);
    }

    // dla robotow track i postument - oblicz chwytak
    if ((robot_name == ROBOT_IRP6_ON_TRACK) || (robot_name == ROBOT_IRP6_POSTUMENT))
    {
        reply.arm.coordinate_def.gripper_reg_state = servo_gripper_reg_state;
        reply.arm.frame_def.gripper_coordinate = current_joints[gripper_servo_nr];
    }

}
; // end: edp_irp6s_effector::arm_frame_2_frame
/*--------------------------------------------------------------------------*/

void edp_irp6s_effector::set_rmodel (c_buffer *instruction)
{}
;                    // zmiana narzedzia
void edp_irp6s_effector::get_rmodel (c_buffer *instruction)
{}
;     

void edp_irp6s_effector::master_joints_and_frame_download (void)
{ // by Y
    pthread_mutex_lock( &edp_irp6s_effector_mutex );
    // przepisanie danych na zestaw lokalny dla edp_master
    for (int i=0; i < number_of_servos; i++)
    {
        current_motor_pos[i]=global_current_motor_pos[i];
        current_joints[i]=global_current_joints[i];
    }
    copy_frame(servo_current_frame_wo_tool, global_current_frame_wo_tool);
    pthread_mutex_unlock( &edp_irp6s_effector_mutex );
};





/*--------------------------------------------------------------------------*/
void edp_irp6s_effector::create_threads ()
{
    edp_irp6s_and_conv_effector::create_threads();
}

/*--------------------------------------------------------------------------*/

void edp_irp6s_effector::get_arm_position (bool read_hardware, c_buffer *instruction)
{} // odczytanie pozycji ramienia

// sprawdza stan EDP zaraz po jego uruchomieniu
void edp_irp6s_effector::servo_joints_and_frame_actualization_and_upload (void)
{}
