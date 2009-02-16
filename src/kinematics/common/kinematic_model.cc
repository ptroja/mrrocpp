// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota - klasa abstrakcyjna.
//				- definicja czesci metod klasy
//
// Autor:		tkornuta
// Data:		17.03.2007
// ------------------------------------------------------------------------

// Klasa kinematic_model.
#include "kinematics/common/kinematic_model.h"

/* ------------------------------------------------------------------------
  Konstruktor. Domyslnie wszystkie obliczenia sa wlaczone.
 ------------------------------------------------------------------------ */
kinematic_model::kinematic_model(void)
{
    kinematic_model_label = NULL;
    // Flaga - czy przeliczac do globalnego ukladu odniesienia.
    global_frame_computations = false;
    // Flaga - czy wykonywac przeliczenia zwiazane z narzedziami.
    attached_tool_computations = false;
    // Flaga - uzywac lokalnych korektorow.
    local_corrector_computations = false;
}


/* ------------------------------------------------------------------------
  Przeliczenie polozenia koncowki zwiazane z dolaczonym narzedziem.
 ------------------------------------------------------------------------ */
void kinematic_model::attached_tool_transform(Homog_matrix& current_end_effector_matrix)
{
    current_end_effector_matrix *= tool;
}


/* ------------------------------------------------------------------------
  Przeliczenie polozenia koncowki zwiazane z dolaczonym narzedziem - transformacja odwrotna.
 ------------------------------------------------------------------------ */
void kinematic_model::attached_tool_inverse_transform(Homog_matrix& desired_end_effector_matrix)
{
    desired_end_effector_matrix *= (!tool);
}


/* ------------------------------------------------------------------------
 Przeliczenie bazy manipulatora w globalnym ukladzie odniesienia.
 ------------------------------------------------------------------------ */
void kinematic_model::global_frame_transform(Homog_matrix& current_end_effector_matrix)
{
    current_end_effector_matrix = (global_base * current_end_effector_matrix);
}


/* ------------------------------------------------------------------------
 Przeliczenie bazy manipulatora w globalnym ukladzie odniesienia - transformacja odwrotna.
 ------------------------------------------------------------------------ */
void kinematic_model::global_frame_inverse_transform(Homog_matrix& desired_end_effector_matrix)
{
    desired_end_effector_matrix = ((!global_base) * desired_end_effector_matrix);
}


/* ------------------------------------------------------------------------
 Poprawa polozenia koncowki przy uzyciu macierzy korekcji lokalnej.
 ------------------------------------------------------------------------ */
void kinematic_model::local_corrector_transform(Homog_matrix& current_end_effector_matrix)
{
    //  	std::cout<<" local_corrector_transform: przed \n"<<current_end_effector_matrix<<std::endl;
    double d[6];
    current_end_effector_matrix.get_xyz_euler_zyz(d);
    // Przeksztalcenie polozenia do z XYZ_Euler_ZYZ odpowiedniej postaci.
    double x[6];
    x[0] = d[0] * 1000;
    x[1] = d[1] * 1000;
    x[2] = d[2] * 1000;
    x[3] = d[3] * h;
    x[4] = d[4] * h;
    x[5] = d[5] * h;

    double z[6];
    // Obliczenie poprawionych wspolczynnikow.
    for(int i=0; i<6; i++)
    {
        z[i] =0;
        for(int j=0; j<6; j++)
        {
            z[i] += U[i][j] * x[j];
        }
        ;//: for
        z[i] += V[i];
    }

    // Przeksztalcenie poprawionego polozenia do postaci XYZ_Euler_ZYZ.
    d[0] = z[0] / 1000;
    d[1] = z[1] / 1000;
    d[2] = z[2] / 1000;
    d[3] = z[3] / h;
    d[4] = z[4] / h;
    d[5] = z[5] / h;
    // Zapamietanie poprawionego polozenia.
    current_end_effector_matrix.set_xyz_euler_zyz(d[0], d[1], d[2], d[3], d[4], d[5]);
    //  	std::cout<<" local_corrector_transform: poprawione \n"<<current_end_effector_matrix<<std::endl;
}

/* ------------------------------------------------------------------------
 Poprawa polozenia koncowki przy uzyciu macierzy korekcji lokalnej - transformacja odwrotna.
 ------------------------------------------------------------------------ */
void kinematic_model::local_corrector_inverse_transform(Homog_matrix& desired_end_effector_matrix)
{
    //  	std::cout<<" local_corrector_inverse_transform: przed \n"<<desired_end_effector_matrix<<std::endl;
    double d[6];
    desired_end_effector_matrix.get_xyz_euler_zyz(d);
    // Przeksztalcenie polozenia do z XYZ_Euler_ZYZ odpowiedniej postaci.
    double z[6];
    z[0] = d[0] * 1000;
    z[1] = d[1] * 1000;
    z[2] = d[2] * 1000;
    z[3] = d[3] * h;
    z[4] = d[4] * h;
    z[5] = d[5] * h;

    double x[6];
    double zminV[6];
    // Obliczenie orygnalnych  wspolczynnikow.
    for(int i=0; i<6; i++)
        zminV[i] = z[i] - V[i];
    for(int i=0; i<6; i++)
    {
        x[i] = 0;
        for(int j=0; j<6; j++)
        {
            x[i] += inv_U[i][j] * zminV[j];
        }
    }

    // Przeksztalcenie pierwotnego polozenia do postaci XYZ_Euler_ZYZ.
    d[0] = x[0] / 1000;
    d[1] = x[1] / 1000;
    d[2] = x[2] / 1000;
    d[3] = x[3] / h;
    d[4] = x[4] / h;
    d[5] = x[5] / h;
    // Zapamietanie polozenia.
    desired_end_effector_matrix.set_xyz_euler_zyz(d[0], d[1], d[2], d[3], d[4], d[5]);
    //  	std::cout<<" local_corrector_inverse_transform: oryginalne \n"<<desired_end_effector_matrix<<std::endl;
}



/* ------------------------------------------------------------------------
  Przeliczenie polozenia ze wspolrzednych wewnetrznych na wspolrzedne zewnetrzne (i2e - internal to external)

-> Dodac opis kolejnosci obliczen.

 ------------------------------------------------------------------------ */
void kinematic_model::i2e_transform(const double* local_current_joints, frame_tab* local_current_end_effector_frame)
{

    // Rozwiazanie prostego zagadnienia kinematyki.
    direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);

    // Stworzenie macierzy, ktora bedzie uzywana w dalszych obliczeniach.
    Homog_matrix local_current_end_effector_matrix (*local_current_end_effector_frame);

    // Obliczenia zwiazane z przeksztalceniami do globalnego ukladu odniesienia.
    if(global_frame_computations)
        global_frame_transform(local_current_end_effector_matrix);

    // Obliczenia zwiazane z macierza korekcji lokalnej.
    if(local_corrector_computations)
        local_corrector_transform(local_current_end_effector_matrix);

    // Przeksztalcenie polozenia koncowki zwiazane z uzytym narzedziem (chwytakiem).
    if (attached_tool_computations)
        attached_tool_transform(local_current_end_effector_matrix);

    // Przepisanie wyniku z macierzy.
    local_current_end_effector_matrix.get_frame_tab(*local_current_end_effector_frame);
}


/* ------------------------------------------------------------------------
  Przeliczenie polozenia ze wspolrzednych zewnetrznych na wspolrzedne zewnetrzne (e2i - external to internal).

-> Dodac opis kolejnosci obliczen.

 ------------------------------------------------------------------------ */
void kinematic_model::e2i_transform(double* local_desired_joints, double* local_current_joints, frame_tab* local_desired_end_effector_frame)
{
    // Stworzenie macierzy, ktora bedzie uzywana w dalszych obliczeniach.
    Homog_matrix local_desired_end_effector_matrix (*local_desired_end_effector_frame);

    // Przeksztalcenie odwrotne polozenia koncowki zwiazane z uzytym narzedziem (chwytakiem).
    if (attached_tool_computations)
        attached_tool_inverse_transform(local_desired_end_effector_matrix);

    // Obliczenia odwrotne zwiazane z macierza korekcji lokalnej.
    if (local_corrector_computations)
        local_corrector_inverse_transform(local_desired_end_effector_matrix);

    // Obliczenia odwrotne zwiazane z przeksztalceniami do globalnego ukladu odniesienia.
    if(global_frame_computations)
        global_frame_inverse_transform(local_desired_end_effector_matrix);

    // Przepisanie wyniku z macierzy.
    local_desired_end_effector_matrix.get_frame_tab(*local_desired_end_effector_frame);

    // Rozwiazanie odwrotnego zagadnienia kinematyki.
    inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_frame);

    /*	printf("Ramka w e2i\n");
    printf("%lf; %lf; %lf; %lf \n",(*local_desired_end_effector_frame)[0][0], (*local_desired_end_effector_frame)[0][1], (*local_desired_end_effector_frame)[0][2], (*local_desired_end_effector_frame)[0][3]);
    printf("%lf; %lf; %lf; %lf \n",(*local_desired_end_effector_frame)[1][0], (*local_desired_end_effector_frame)[1][1], (*local_desired_end_effector_frame)[1][2], (*local_desired_end_effector_frame)[1][3]);
    printf("%lf; %lf; %lf; %lf \n",(*local_desired_end_effector_frame)[2][0], (*local_desired_end_effector_frame)[2][1], (*local_desired_end_effector_frame)[2][2], (*local_desired_end_effector_frame)[2][3]);
    printf("%lf; %lf; %lf; %lf \n",(*local_desired_end_effector_frame)[3][0], (*local_desired_end_effector_frame)[3][1], (*local_desired_end_effector_frame)[3][2], (*local_desired_end_effector_frame)[3][3]);
      */

}


/* ------------------------------------------------------------------------
  Zwraca etykiete modelu kinematycznego.
 ------------------------------------------------------------------------ */
char* kinematic_model::get_kinematic_model_label(void)
{
    return kinematic_model_label;
}

/* ------------------------------------------------------------------------
  Ustawia etykiete modelu kinematycznego.
 ------------------------------------------------------------------------ */
void kinematic_model::set_kinematic_model_label(const char * _label)
{
    if (kinematic_model_label != NULL)
        delete(kinematic_model_label);
    kinematic_model_label = new char[strlen(_label)];
    kinematic_model_label = strcpy(kinematic_model_label, _label);
}
