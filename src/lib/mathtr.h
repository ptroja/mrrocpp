// ------------------------------------------------------------------------
// Proces:		-
// Plik:			mathtr.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasy K_vector, Homog_matrix, Ft_v_vector,  Jacobian_matrix
//				- definicja klas
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------


#ifndef __MATHTR_H
#define __MATHTR_H

#include <iostream>
#include <math.h>
#include <string.h>

#include "lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {


#define delta_m (M_PI - 3.14154)
#define DEGREES_TO_RADIANS 57.295780

#define zero_eps 1.0E-4

#define ALFA_SENSITIVITY 0.00001


// Sprowadzenie wartosci kata do przedzialu <-pi,pi>
double reduce(double angle);

// deklaracje zapowiadajace
class Homog_matrix;
class Ft_v_vector;
class Ft_v_tr;

// klasa reprezentujaca wektor w kartezjaskim ukaladzie odniesienia
class K_vector
{
private:
	double w[3];

public:
	friend class Homog_matrix;						// klasa Homog_matrix musi miec dostep do prywatnych
															// skladnikow klasy vector

	K_vector ();												// konstruktor domniemany: [0, 0, 0]
	K_vector (double t[3]);								// utworzenie wektora na podstawie tablicy
	K_vector (double x, double y, double z);			// utworzenie wektora na podstawie tablicy
	K_vector (const K_vector &);							// konstruktor kopiujacy


	// Ustawienie elementu wektora.
	void set_value(int i, const double value);
	// Zwrocenie elementu wektora.
	void get_value(int i, double &value) const;
	// Zwrocenie elementu wektora.
	double get_value(int i) const;
	double get_length() const;
	void normalize();

	void to_table(double tablica[3]) const;			// przepisanie zawartosci do tablicy

	K_vector & operator=(const K_vector &);			// operator przypisania
	K_vector & operator=(const double[3]);			// przypisanie tablicy na wektor			- by Slawek Bazant
	K_vector operator+(const K_vector &) const;		// dodawanie wektorow
	K_vector operator-(const K_vector &) const;		// odejmowanie wektorow					- by Slawek Bazant
	K_vector operator*(const K_vector &) const;		// iloczyn wektorowy						- by Slawek Bazant
	K_vector operator*(double) const;					// skalowanie wektora						- by Slawek Bazant
	void operator+=(const K_vector &);				// dodanie wektora podanego jako agument do aktualnego
	void operator*=(const double);					// skalowanie wektora						- by Slawek Bazant

	// in theory, the RHS operator
    double operator[](const int i) const;
    // in theory, the LHS operator
    double& operator[](const int i);

	friend std::ostream& operator<<(std::ostream & s, K_vector & w);	// operator wypisania

};// end class vector


// Klasa reprezentujaca macierz transformacji.
class Homog_matrix
{



private:
	// Zmienna przechowujaca parametry macierzy jednorodnej.
	frame_tab matrix_m;

public:
	// Klasa Ft_v_tr musi miec dostep do prywatnych skladnikow klasy Homog_matrix.
	friend class Ft_v_tr;

	enum POSE_SPECIFICATION { MTR_XYZ_ANGLE_AXIS, MTR_XYZ_EULER_ZYZ, MTR_MECH_XYZ_EULER_ZYZ, MTR_XYZ_RPY};

	// Konstruktor domniemany - tworzy macierz jednostkowa.
	Homog_matrix();
	// Stworzenie macierzy na podstawie zawartosci tablicy.
	Homog_matrix(const frame_tab);
	// Konstruktor kopiujacy.
	Homog_matrix(const Homog_matrix &);
	// Utworzenie macierzy przesuniecia o [x, y, z], R - jednostkowa.
	Homog_matrix(double x, double y, double z);
	// Utworzenie macierzy obrotu o male katy wzgledem 3 osi.
	Homog_matrix(K_vector versor_x, K_vector versor_y, K_vector versor_z, K_vector angles);
	// Utworzenie macierzy obrotu o male katy
	Homog_matrix(K_vector angles);
	// Utworzenie macierzy jednorodnej na podstawie podanej macierzy obrotu r i wektora przesuniecia t.
	Homog_matrix(double r[3][3], double t[3]);
	// Utworzenie macierzy jednorodnej na podstawie jej 12 elementow (notacja z Craiga)
	Homog_matrix (double r11, double r12, double r13, double t1, double r21, double r22, double r23, double t2, double r31, double r32,
	 double r33, double t3);
	// Utworzenie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS.
	Homog_matrix(double kx, double ky, double kz, double alfa, double x, double y, double z);
	// Utworzenie macierzy jednorodnej na podstawie rozkazu w formie XYZ_EULER_ZYZ.
	Homog_matrix (POSE_SPECIFICATION mtr_ps, double x, double y, double z, double alfa, double beta, double gamma);

	// Konstruktor, ktory wypelnienia wspolczynniki macierzy na podstawie danych w formie
	Homog_matrix(POSE_SPECIFICATION mtr_ps, const K_vector axis_with_angle, const K_vector translation);

	// Konstruktor, ktory wypelnienia wspolczynniki macierzy na podstawie danych w formie
	Homog_matrix(POSE_SPECIFICATION mtr_ps, const Ft_v_vector translation_and_axis_with_angle);

	// Konstruktor, ktory wypelnienia wspolczynniki macierzy na podstawie danych w formie
	Homog_matrix(POSE_SPECIFICATION mtr_ps, const double t[6]);

	Homog_matrix return_with_with_removed_translation() const;
	Homog_matrix return_with_with_removed_rotation() const;

	// Zwrocenie tablicy zawierajacej dane macierzy jednorodnej.
	void get_frame_tab(frame_tab frame) const;
	// Ustawienie tablicy zawierajacej dane macierzy jednorodnej.
	void set_frame_tab(const frame_tab frame);

     // Przeksztalcenie do formy XYZ_EULER_ZYZ i zwrocenie w tablicy.
	void get_xyz_euler_zyz(double t[6]) const;
	// Przeksztalcenie do formy XYZ_EULER_ZYZ dla robota IRP-6_MECHATRONIKA i zwrocenie w tablicy.
	void get_mech_xyz_euler_zyz(double t[6]) const;
	// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_EULER_ZYZ.
	void set_xyz_euler_zyz(double x, double y, double z, double alfa, double beta, double gamma);
	void set_xyz_euler_zyz(const double t[6]);
	// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_EULER_ZYZ dla robota IRP-6_MECHATRONIKA
	void set_mech_xyz_euler_zyz(double x, double y, double z, double alfa, double beta, double gamma);

     // Przeksztalcenie do formy XYZ_RPY (rool pitch yaw) i zwrocenie w tablicy.
	void get_xyz_rpy(double t[6]) const;
	// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_RPY.
	void set_xyz_rpy(double x, double y, double z, double alfa, double beta, double gamma);


	// Przeksztalcenie do formy XYZ_ANGLE_AXIS i zwrocenie w tablicy.
	void get_xyz_angle_axis(double t[6]) const;
	void get_xyz_angle_axis(K_vector& axis_with_angle, K_vector& translation) const;
	void get_xyz_angle_axis(Ft_v_vector& translation_and_axis_with_angle) const;

	// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
	void set_xyz_angle_axis(double kx, double ky, double kz, double alfa, double x, double y, double z);
	void set_xyz_angle_axis(double kx, double ky, double kz, double x, double y, double z); // kat wliczony w os
	void set_xyz_angle_axis(const K_vector axis_with_angle, const K_vector translation);  // kat wliczony w os
	void set_xyz_angle_axis(const Ft_v_vector translation_and_axis_with_angle);  // kat wliczony w os
	void set_xyz_angle_axis(const double t[6]);  // kat wliczony w os

	// Operacje na kwaternionach
	void set_xyz_quaternion(double eta, double eps1, double eps2, double eps3, double x, double y, double z);
	void get_xyz_quaternion(double t[7]) const;


	// Zwraca obecny wektor translacji.
	void get_translation_vector(double t[3]) const;

	// wyzerowanie wektora translacji.
	void remove_translation();

	// wstawienie jedynek na diagonalii rotacji
	void remove_rotation();

	// Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
	void set_translation_vector(double t[3]);

	void set_translation_vector(double x, double y, double z);

	void set_translation_vector(const K_vector xyz);

	void set_translation_vector(const Homog_matrix &wzor);

	// Zwrocenie macierzy rotacji.
	void get_rotation_matrix(double r[3][3]) const;
	// Ustawienie macierzy rotacji. Wektor translacji pozostaje niezmieniony.
	void set_rotation_matrix(double r[3][3]);

	void set_rotation_matrix(const Homog_matrix &wzor);

	// Ustawienie elementu macierzy.
	void set_value(int i, int j, const double value);
	// Zwrocenie elementu macierzy.
	void get_value(int i, int j, double &value) const;
	// Zwrocenie elementu macierzy.
	double get_value(int i, int j) const;


	// Operator przypisania.
	Homog_matrix & operator=(const Homog_matrix &);
	// Mnozenie macierzy.
	Homog_matrix operator* (const Homog_matrix &) const;
	// Odwracanie macierzy.
	Homog_matrix operator!() const;
	// Mnozenie macierzy i przypisanie wyniku.
	void operator*= (const Homog_matrix &);

	// operatory sluzace do przeksztalcania wektorow
	K_vector operator*(const K_vector &) const;
	K_vector operator*(const double tablica[3]) const;

	// operatory prownania macierzy jednorodnych
	int operator==(const Homog_matrix &) const;
	int operator!=(const Homog_matrix &) const;

	// operator wypisania
	friend std::ostream& operator<<(std::ostream &, Homog_matrix &);

	// funkcja sprawdzajaca czy macierz jest macierza jednorodna
	int is_valid() const;

	// Kopiowanie macierzy jednorodnej do DEST z SOURCE.
	inline static void copy_frame_tab(frame_tab destination_frame,   frame_tab source_frame)
	{
		memcpy(destination_frame, source_frame, sizeof(frame_tab));
	};//: copy_frame

	// Kopiowanie macierzy jednorodnej w postaci XYZ_ANGLE_AXIS do DEST z SOURCE.
	inline static void copy_xyz_angle_axis(double  destination_xyz_angle_axis[6], double source_xyz_angle_axis[6])
	{
		memcpy(destination_xyz_angle_axis, source_xyz_angle_axis, 6*sizeof(double));
	};//: copy_xyz_angle_axis

	// Kopiowanie macierzy jednorodnej w postaci XYZ_EULER_ZYZ do DEST_FRAME z SOURCE_FRAME.
	inline static void copy_xyz_euler_zyz(double destination_xyz_euler_zyz[6],   double source_xyz_euler_zyz[6])
	{
		memcpy(destination_xyz_euler_zyz, source_xyz_euler_zyz, 6*sizeof(double));
	};//: copy_xyz_euler_zyz

};// end class Homog_matrix


// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_v_vector
{
private:
	double w[6];
public:
	friend class Ft_v_tr;						// klasa Ft_v_tr musi miec dostep do prywatnych
												// skladnikow klasy Ft_v_vector
     friend class Jacobian_matrix;			//Klasa Jacobian_matrix ma miec dostep do skladowych - Sibi

	Ft_v_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Ft_v_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Ft_v_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	Ft_v_vector(const K_vector force, const K_vector torque);

	Ft_v_vector(const Ft_v_vector &);								// konstruktor kopiujacy

	void set_values(const double t[6]);										// wypelnienie wektora na podstawie podanej tablicy
	void set_values(double fx, double fy, double fz, double tx, double ty, double tz);

	// Ustawienie elementu wektora.
	void set_value(int i, const double value);
	// Zwrocenie elementu wektora.
	void get_value(int i, double &value) const;
	// Zwrocenie elementu wektora.
	double get_value(int i) const;

	//Sibi
	 //Wektor predkosci jako odleglosc dwuch pozycji zadanych w postaci ramek
	void position_distance(frame_tab* local_current_end_effector_frame, frame_tab* local_desired_end_effector_frame);


	K_vector get_force_K_vector()  const;
	K_vector get_torque_K_vector()  const;

	// Wyspisanie na ekran wektora
	void wypisz_wartosc_na_konsole() const;

	// in theory, the RHS operator
    double operator[](const int i) const;
      // in theory, the LHS operator
    double& operator[](const int i);

	// Odwracanie macierzy.
	Ft_v_vector operator!() const;
	Ft_v_vector operator-() const;
	Ft_v_vector & operator=(const Ft_v_vector &);			// operator przypisania
	Ft_v_vector operator+(const Ft_v_vector &) const;
	Ft_v_vector operator-(const Ft_v_vector &) const;
	Ft_v_vector operator*(double) const;					// skalowanie wektora
	void operator+=(const Ft_v_vector &);


	void to_table(double tablica[6]) const;					// przepisanie wektora do tablicy podanej jako argument

	friend std::ostream& operator<<(std::ostream & s, Ft_v_vector & w);		// operator wypisania

	//Sibi
	//Wyciagniecie max elementu z wektora
	double max_element ();	//wyciagniecie maksymalnego elementu wektora

};// end class Ft_v_vector


// klasa reprezentujaca macierz transformacji odczytow sily do innego ukladu odniesienia
class Ft_v_tr
{
public:
	enum VARIANT { FT, V, NOT_SET};

private:
	double matrix_m[6][6];												// zmienna przechowujaca parametry macierzy
	Homog_matrix base_frame;										// bazowy trojscian z konstruktora
	 VARIANT variant;

public:

	Ft_v_tr ();																	// kostruktor domniemany
//	Ft_v_tr(VARIANT variant_l);
	Ft_v_tr (const Homog_matrix &, VARIANT variant_l);
	Ft_v_tr(const Ft_v_tr &);												// konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p);		// ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	Ft_v_tr operator* (const Ft_v_tr & m) const;
	Ft_v_tr operator!() const;

	Ft_v_tr & operator = (const Ft_v_tr &);									// operator przypisania
	Ft_v_vector operator*(const Ft_v_vector &) const;					// mnozenie wektora


	friend std::ostream&  operator<<(std::ostream & strumien, Ft_v_tr &);		// operator wypisania

};// end class Ft_v_tr


//Sibi
// klasa reprezentujaca macierz jakobianu 6 na 6
class Jacobian_matrix
{
private:
	double matrix[6][6];												// Miejsce na jakobian

public:
	Jacobian_matrix ();													// kostruktor domniemany

     void irp6_6dof_equations(const Ft_v_vector & w);			//Wzory na jakobian dla Irp-6 o 6 stopniach swobody
     void irp6_6dof_inverse_equations(const Ft_v_vector & w);				//Wzory na odwrotnosc jakobianu dla Irp-6 o 6 stopniach swobody
	 double irp6_6dof_determinant(const Ft_v_vector & w);					//Wzory na wyznacznik jaokbianu dla Irp-6 o 6 stopniach swobody

     void jacobian_transpose();										//Wyznaczenie transpozycji jakobianu
     void wypisz();														//Wypisanie zawartosci macierzy na konsole
     void to_table(double tablica[6][6]) const;						// przepisanie elementw jakobianu do tablicy[6][6] podanej jako argument

	Ft_v_vector jacobian_inverse_gauss(const Ft_v_vector & dist);				//Rozwiazanie ukladu rownan AX=Y (A, Y - zadane)
																								//za pomoca metody eliminacji Gaussa
	Ft_v_vector operator* (const Ft_v_vector & w) const;		//Przeciazenie operacji mnozenia dla jakobianu i wektora

};// end class Jacobian_matrix

} // namespace lib
} // namespace mrrocpp

#endif
