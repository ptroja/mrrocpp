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


#ifndef __HOMOG_MATRIX_H
#define __HOMOG_MATRIX_H

#include <iostream>
#include <math.h>
#include <string.h>

#include "lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {


// Klasa reprezentujaca macierz transformacji.
class Homog_matrix
{



private:
	// Zmienna przechowujaca parametry macierzy jednorodnej.
	frame_tab matrix_m;

public:
	// Klasa Ft_v_tr musi miec dostep do prywatnych skladnikow klasy Homog_matrix.
	friend class Ft_v_tr;
	friend class Ft_tr;
	friend class V_tr;

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

	double* operator[](const int i);

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



} // namespace lib
} // namespace mrrocpp

#endif
