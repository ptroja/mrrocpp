#ifndef _matrix_h
#define _matrix_h

#include <valarray>
#include <iterator>
#include <iostream>
#include <vector>

namespace NurbsLibImp {
using namespace std;
// forward declarations to allow friend declarations:
template<class T> class Slice_iter;
template<class T> bool operator==(const Slice_iter<T>&, const Slice_iter<T>&);
template<class T> bool operator!=(const Slice_iter<T>&, const Slice_iter<T>&);
template<class T> bool operator< (const Slice_iter<T>&, const Slice_iter<T>&);

template<class T> class Slice_iter {
	valarray<T>* v;
	slice s;
	size_t curr;	// index of current element

	T& ref(size_t i) const { return (*v)[s.start()+i*s.stride()]; }
public:
	Slice_iter(valarray<T>* vv, slice ss) :v(vv), s(ss), curr(0) { }

//	slice& Slice() {return s; }

	Slice_iter end() const
	{
		Slice_iter t = *this;
		t.curr = s.size();	// index of last-plus-one element
		return t;
	}

	Slice_iter& operator++() { curr++; return *this; }
	Slice_iter operator++(int) { Slice_iter t = *this; curr++; return t; }

	T& operator[](size_t i) { return ref(i); }		// C style subscript
	T& operator()(size_t i) { return ref(i); }		// Fortran-style subscript
	T& operator*() { return ref(curr); }			// current element

	friend bool operator==<>(const Slice_iter& p, const Slice_iter& q);
	friend bool operator!=<>(const Slice_iter& p, const Slice_iter& q);
	friend bool operator< <>(const Slice_iter& p, const Slice_iter& q);

};

//-------------------------------------------------------------



// forward declarations to allow friend declarations:
template<class T> class Cslice_iter;
template<class T> bool operator==(const Cslice_iter<T>&, const Cslice_iter<T>&);
template<class T> bool operator!=(const Cslice_iter<T>&, const Cslice_iter<T>&);
template<class T> bool operator< (const Cslice_iter<T>&, const Cslice_iter<T>&);


template<class T> class Cslice_iter
{
	valarray<T>* v;
	slice s;
	size_t curr; // index of current element
	const T& ref(size_t i) const { return (*v)[s.start()+i*s.stride()]; }
public:
	Cslice_iter(valarray<T>* vv, slice ss): v(vv), s(ss), curr(0){}
	Cslice_iter end() const
	{
		Cslice_iter t = *this;
		t.curr = s.size(); // index of one plus last element
		return t;
	}
	Cslice_iter& operator++() { curr++; return *this; }
	Cslice_iter operator++(int) { Cslice_iter t = *this; curr++; return t; }

	const T& operator[](size_t i) const { return ref(i); }
	const T& operator()(size_t i) const { return ref(i); }
	const T& operator*() const { return ref(curr); }

	friend bool operator==<>(const Cslice_iter& p, const Cslice_iter& q);
	friend bool operator!=<>(const Cslice_iter& p, const Cslice_iter& q);
	friend bool operator< <>(const Cslice_iter& p, const Cslice_iter& q);

};

class Matrix {
private:
	valarray<double> *v; // przechowuje elementy kolumnami jak jest opisane w Stroustrup Jezyk C++ 22.4.5
	size_t r, c; //r - liczba wierszy, c - liczba kolumn

	Matrix operator=( Matrix&); //nie zaimplementowany; deklaracja jest potrzebna, bo w przeciwnym razie
		//automatycznie zostanie wygenerowany bledny operator, ktorego ktos moglby przypadkiem uzyc

public:
	Matrix(size_t max_row, size_t max_col);
	Matrix(const Matrix&); //konstruktor kopiujacy
	Matrix& operator<<(const Matrix&);
	Matrix& operator=(const Matrix&);
	~Matrix();

	size_t size() const { return r*c;}

//	size_t size_x() const {return r;}
//	size_t size_y() const {return c;}
	size_t rows() const {return r;}
	size_t cols() const {return c;}


	Slice_iter<double> row(size_t i);
	Cslice_iter<double> row(size_t i) const;

	Slice_iter<double> column(size_t i);
	Cslice_iter<double> column(size_t i) const;

	Slice_iter<double> operator[] (size_t i) {return row(i);}
	Cslice_iter<double> operator[] (size_t i) const {return row(i);}

	double& operator()(size_t row_nr, size_t col_nr) {
		if (col_nr>=c) return column(col_nr)[row_nr]; return row(row_nr)[col_nr];}	//z komunikowaniem bledu zakresu
	double operator()(size_t row_nr, size_t col_nr) const {
		if (col_nr>=c) return column(col_nr)[row_nr]; return row(row_nr)[col_nr];}

	Matrix& operator*=(double);
	Matrix& operator*=(const Matrix&);

	void resize(size_t rows, size_t cols) {(*v).resize(rows*cols); c=cols; r=rows; return;} //nie zachowuje wartoci elementow

	valarray<double>& array(){return *v;}
};

ostream& operator<<(ostream& , const Matrix& ) ;
ostream& operator<<(ostream& , const vector<double>& );

inline Slice_iter<double> Matrix::row(size_t i) {
	if (i>=rows()) cerr<<"Matrix::row: przekroczony zakres "<<i<<">"<<rows()-1<<".\n"<<flush;
	return Slice_iter<double>(v, slice(i, c, r)); }

inline Cslice_iter<double> Matrix::row(size_t i) const {
	if (i>=rows()) cerr<<"Matrix::row: przekroczony zakres "<<i<<">"<<rows()-1<<".\n"<<flush;
	return Cslice_iter<double>(v, slice(i, c, r)); }

inline Slice_iter<double> Matrix::column(size_t i) {
	if (i>=cols()) cerr<<"Matrix::column: przekroczony zakres "<<i<<">"<<cols()-1<<".\n"<<flush;
	return Slice_iter<double>(v, slice(i*r, r, 1)); }

inline Cslice_iter<double> Matrix::column(size_t i) const {
	if (i>=cols()) cerr<<"Matrix::column: przekroczony zakres "<<i<<">"<<cols()-1<<".\n"<<flush;
	return Cslice_iter<double>(v, slice(i*r, r, 1)); }


//--------Mat-------LUMatrix----------------------------------------------------------
int solve(const Matrix& AX, const Matrix& BX, Matrix& XX);

class LUMatrix : public Matrix
  {
  private:
    vector<int> pivot_ ;

  public:
    LUMatrix(const Matrix& a): Matrix(a.rows(),a.cols()),pivot(pivot_) {decompose(a) ; }

    LUMatrix& decompose(const Matrix &a);

    void resize(const int r, const int c) { Matrix::resize(r,c) ; pivot_.resize(r) ; }
    Matrix inverse();
//    void inverseIn(Matrix&);
    Matrix inverseIn();
    const vector<int>& pivot ;

  };

} //namespace NurbsLibImp

// ############################################################################
// -----------------------------------------------------------------
// 			 MACIERZ
// -----------------------------------------------------------------

class matrix;

class matrix {
  private:
    double **ptr;
    	int rows;
		int columns;

  public:
    matrix ();
    matrix (int r, int c);
    matrix ( const matrix &  srcMatrix );    // konstruktor kopiujacy
   ~matrix ();					  // destruktor obiektow Matrix

   double& operator ()( int, int );
};

#endif /* _matrix_h */
