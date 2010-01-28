#ifndef _matrix_h
#define _matrix_h

// ############################################################################
// -----------------------------------------------------------------
// 			 MACIERZ
// -----------------------------------------------------------------

class matrix;

class matrix
{
private:
	double **ptr;
	int rows;
	int columns;

public:
	matrix();
	matrix(int r, int c);
	matrix(const matrix & srcMatrix); // konstruktor kopiujacy
	~matrix(); // destruktor obiektow Matrix

	double& operator ()(int, int);
};

#endif /* _matrix_h */
