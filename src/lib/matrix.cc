#include "lib/matrix.h"

//---------Slice_iter--------------------------------------------------------------
using namespace NurbsLibImp;

template<class T>
bool operator==(const Slice_iter<T>& p, const Slice_iter<T>& q)
{
	return p.curr==q.curr
		&& p.s.stride()==q.s.stride()
		&& p.s.start()==q.s.start();
}

template<class T>
bool operator!=(const Slice_iter<T>& p, const Slice_iter<T>& q)
{
	return !(p==q);
}

template<class T>
bool operator<(const Slice_iter<T>& p, const Slice_iter<T>& q)
{
	return p.curr<q.curr
		&& p.s.stride()==q.s.stride()
		&& p.s.start()==q.s.start();
}

//---------Cslice_iter--------------------------------------------------------------

template<class T>
bool operator==(const Cslice_iter<T>& p, const Cslice_iter<T>& q)
{
	return p.curr==q.curr
		&& p.s.stride()==q.s.stride()
		&& p.s.start()==q.s.start();
}

template<class T>
bool operator!=(const Cslice_iter<T>& p, const Cslice_iter<T>& q)
{
	return !(p==q);
}

template<class T>
bool operator<(const Cslice_iter<T>& p, const Cslice_iter<T>& q)
{
	return p.curr<q.curr
		&& p.s.stride()==q.s.stride()
		&& p.s.start()==q.s.start();
}


//------Matrix---------------------------------------------------------------------


Matrix::Matrix(size_t max_row, size_t max_col) {
	r=max_row;
	c=max_col;
	v = new valarray<double> (0.0, r*c);
//	cout<<"Matrix::Matrix("<<x<<","<<y<<"): konstrukcja zakonczona.\n";
}

Matrix::Matrix(const Matrix& m) {
	r=m.rows();
	c=m.cols();
	v = new valarray<double> (r*c);
	(*v) = *(m.v); }

Matrix& Matrix::operator<<(const Matrix& m) { //dziala jak operator=, ale nie sprawdza wymiaru macierzy na ktora przypisuje
	delete v;
	r=m.rows(); c=m.cols();
	v = new valarray<double> (r*c); 
	(*v) = *(m.v);
	return *this; }

Matrix& Matrix::operator=(const Matrix& m) {
	if (c!=m.cols() || r!=m.rows()) {
		cerr<<"Matrix: wrong size of matrix in m1=m2\n";
		r=m.rows(); c=m.cols();
		delete v;
		v = new valarray<double> (r*c); }
	(*v) = *(m.v);
//	for (size_t i = 0; i<(*(m.v)).size(); i++) 
//		cout<<" m.v:"<<(*(m.v))[i]<<flush;
//	for (size_t i = 0; i<(*v).size(); i++) 
//		cout<<"   v:"<<(*v)[i]<<flush;
//	cout<<"\n";
	return *this; }

Matrix::~Matrix() {
	delete v; }
	
double iloczyn (const Cslice_iter<double>& v1, const valarray<double>& v2) {
	double wyn=0;
	for (size_t i = 0; i<v2.size(); i++) 	wyn+=v1[i]*v2[i];
	return wyn; }

	
valarray<double> operator* (const Matrix& m, const valarray<double>& v) {
	if (m.cols()!=v.size()) cerr << "Matrix: wrong number of elements in m*v\n";
	
	valarray<double> wyn(m.rows());
	for (size_t i = 0; i<m.rows(); i++) 	wyn[i] = iloczyn(m.row(i), v);
	return wyn; }


Matrix& Matrix::operator*=(const Matrix& m) { //mozna napisac operator* bez przydzielania pamieci i dopiero operator= z przydzielaniem
	//szybsza wersja - Stoustrup 22.4.7
	if (m.rows()!=cols()) cerr << "Matrix: wrong number of elements in m1*=m2; "<<cols()<<"<>"<<m.rows()<<"\n";
//	if (m.cols()!=cols())
	valarray<double> *v2;
	v2 = new valarray<double> (rows()*m.cols());
// 	for(size_t i=0; i<m.size();i++) cout<<"m*=:"<<(*(m.v))[i];cout<<"\n";
	for (size_t i = 0; i< rows(); i++){  //dla kazdego wiersza pierwszej macierzy
		valarray<double> wyn(0.0, m.cols());
		for (size_t j = 0; j<cols(); j++) { //dla kazdej pary
			for (size_t k = 0; k<m.cols(); k++) //dla kazdej kolumny macierzy drugiej
				wyn[k] += row(i)[j] * m.column(k)[j];
//				cout<<"k:"<<k<<"i:"<<i<<"j:"<<j<<row(i)[j]<<"*"<<m.column(k)[j]<<"\n";}
			(*v2)[slice(i,m.cols(),rows())] = wyn;}} 
 	delete v;
// 	for(size_t i=0; i<(*v2).size();i++) cout<<"*=:"<<(*v2)[i];cout<<"\n";
 	v=v2;
 	c=m.cols(); //r=rows();
 	
	return *this; }
	

Matrix& Matrix::operator*=(double d) {
	(*v) *= d;
	return *this; }
	

ostream& operator<<(ostream& s, const Matrix& m) {
	for (size_t j = 0; j<m.rows(); j++) {
		for (size_t  i = 0; i<m.cols(); i++) 
			s << m.row(j)[i] <<"\t"; 
		s << "\n";}
	return s;
}

ostream& operator<<(ostream& s, const vector<double>& v) {
	for (size_t  i = 0; i<v.size(); i++) 
		s << v[i] <<"\t"; 
	return s;
} 
	
	
//-------Mat-----------------------------------------------------------------------

namespace NurbsLibImp {
	int solve(const Matrix& A, const Matrix& B, Matrix& X){
		   
		if (A.cols()!=B.rows())
			cout<<"solve(A,B,X): wrong matrix sizes cols(A)="<<A.cols()<<" rows(B)="<<B.rows()<<"\n"<<flush;
		if(A.cols()==A.rows()){ 
	    // use LU decomposition to solve the problem
	    LUMatrix lu(A) ;
	//    cout<<"LU matrix constructed.\n"<<lu<<flush;
	    X << lu.inverse(); //A^(-1)
	//    cout<<"A inversed.\n"<<X<<flush;
	    X *= B ;
	//    cout<<"X*=B counted.\n"<<X<<flush;
	  }
	  else{
	    return -1; //SVDMatrix not implemented
	//    SVDMatrix<T> svd(A) ;
	//    return svd.solve(B,X) ;
	  }
	  return 1;
	}
}

	
LUMatrix& LUMatrix::decompose(const Matrix& a) {
	
	int i, j, k, l, kp1, nm1, n;
	double t, q; //T
	double den, ten;
	
	n = a.cols();
	
	resize(n,n) ;
	
	if(a.cols()!=a.rows()){
		#ifdef USE_EXCEPTION
		throw WrongSize2D(a.cols(),a.rows(),0,0) ;
		#else
		//    Error err("LUMatrix::decompose");
		//    err << "The a matrix is not squared!" ;
		//    err.fatal() ;
		#endif
	}
	
	//	lu = a;	 must do it by copying or LUFACT will be recursively called !
	for(i=0;i<n;++i)
		for(j=0;j<n;++j)
			row(i)[j]=a.row(i)[j];//elem(i,j) = a(i,j) ;
	
	//  errval = 0;
	nm1 = n - 1;
	for (k = 0; k < n; k++) 	pivot_[k] = k ;
	
	//  sign = 1 ;
	
	if (nm1 >= 1) {	/* non-trivial problem */
		for (k = 0; k < nm1; k++)	{
			kp1 = k + 1;
			// partial pivoting ROW exchanges
			// -- search over column 
			ten = fabs(a.row(k)[k]);
			l = k;
			for (i = kp1; i < n; i++)	{
				den = fabs(a.row(i)[k]);
				if ( den > ten )	{
					ten = den;
					l = i;	}
				}
			pivot_[k] = l;
			
			if ( row(l)[k] != 0.0 ) {	// nonsingular pivot found 
				if (l != k ){	// interchange needed 
					for (i = k; i < n; i++)	{
						t = row(l)[i] ;
						row(l)[i] = row(k)[i] ;
						row(k)[i] = t ;	}
					//		sign = -sign ;
					}
				q =  row(k)[k];	/* scale row */
				for (i = kp1; i < n; i++)	{
					t = - row(i)[k]/q;
					row(i)[k] = t;
					for (j = kp1; j < n; j++)
						row(i)[j] += t * row(k)[j];	}
			}
			else		/* pivot singular */
			;//	    errval = k;
		}		
	}
	
	pivot_[nm1] = nm1;
	//  if (row(nm1)[nm1] == 0.0)
	//    errval = nm1;  
	return *this;
}


Matrix LUMatrix::inverseIn() 
{
	double ten; //T
	int i, j, k, l, kb, kp1, nm1, n, coln;
	
	if ( cols() != rows() )	{
		#ifdef USE_EXCEPTION
		throw WrongSize2D(rows(),cols(),0,0) ;
		#else
		//      Error error("invm");
		//      error << "matrix inverse, not square: " << rows() << " by " << cols() << endl;
		//      error.fatal();
		#endif
	}
	
	n = coln = cols();
	
	LUMatrix& inv = *this ;
	
	nm1 = n - 1;
	
	// inverse U 
//	for(size_t i=0;i<inv.size();i++) cout<<"Inverseln"<<i<<": "<<inv.array()[i];
//	cout<<"\n";
	for (k = 0; k < n; k++)    {
		inv(k,k) = ten = 1.0 / inv(k,k) ;
		ten = -ten;
		for (i = 0; i < k; i++)	inv(i,k) *= ten;
		kp1 = k + 1;
		if (nm1 >= kp1)	{
			for (j = kp1; j < n; j++)  {
				ten = inv(k,j) ;
				inv(k,j) = 0.0 ;
				for (i = 0; i < kp1; i++)	inv(i,j) += ten * inv(i,k) ;}
		}
    }

  // INV(U) * INV(L)   
	if (nm1 >= 1) {
		valarray<double> work(0.0,n); //Vector<T> work(n) ;
		//error.memory( work.memory() );
		
		for (kb = 0; kb < nm1; kb++){
			k = nm1 - kb - 1;
			kp1 = k + 1;
			for (i = kp1; i < n; i++)  {
				work[i] = inv(i,k) ;
				inv(i,k) = 0.0;   }
			for (j = kp1; j < n; j++)   {
				ten = work[j];
				for (i = 0; i < n; i++)
					inv(i,k) += ten * inv(i,j) ;   }
			l = pivot[k];
			if (l != k)
				for (i = 0; i < n; i++)     {
					ten = inv(i,k) ;
					inv(i,k) = inv(i,l);
					inv(i,l) = ten; }
		} 
	} 
//	inv.array()[0]=inv.array()[0]; //dziwne, ale bez tego gina ostatnie obliczenia i na macierz m przypisywana jest jakas starsza wersja macierzy inv.
//tu pewnie ujawnia³ siê b³¹d w metodzie array(). Wczesniej zwracala kopie obiektu a nie referencje.
	Matrix m(cols(), rows()); //nie wydajne rozwiazanie przekazywania macierzy wyniku
	m.array()=inv.array();
//	cout<<"Inverseln A:"<<m;
//	cout<<".\n";
	return m;
}


Matrix LUMatrix::inverse() 
{
  if ( cols() != rows() )
    {
#ifdef USE_EXCEPTION
      throw WrongSize2D(rows(),cols(),0,0) ;
#else
//      Error error("invm");
//      error << "matrix inverse, not square: " << rows() << " by " << cols() << endl;
      cerr << "matrix inverse, not square: " << cols() << " by " << rows() << endl;
//      error.fatal();
#endif
    } 

//  Matrix* inverse ; 
  Matrix inverse(1,1);
 inverse << inverseIn() ;
// cout<<"LUmatrix::inverse(): returning matrix M("<<inverse.cols()<<","<<inverse.rows()<<")\n"<<flush;
//cout<<"\ninverse: "<<inverse;
  return inverse;
}







// --------------------------------------------------------
// Konstruktor bezparametrowy klasy matrix
// --------------------------------------------------------
matrix::matrix ()
{
  rows =0;
  columns =0;

  ptr = new double *[rows];
  for (int i=0; i<rows; i++)
    ptr[i]=new double[columns];
}
// --------------------------------------------------------
// Konstruktor bezparametrowy klasy matrix
// --------------------------------------------------------
matrix::matrix (int r, int c )
{
  ptr = new double *[rows];
  for (int i=0; i<rows; i++)
    ptr[i]=new double[columns];
}
// --------------------------------------------------------
// konstruktor kopiujacy klasy matrix
// --------------------------------------------------------
matrix::matrix( const matrix & srcMatrix )
{
int i,j;
  ptr = new double *[rows];
  for (i=0; i<rows; i++)
    ptr[i]=new double[columns];
  for (i=0; i<rows; i++)
    for (j=0; j<columns; j++)
      ptr[i][j]=srcMatrix.ptr[i][j];
}
// --------------------------------------------------------
// destruktor klasy matrix
// --------------------------------------------------------
matrix::~matrix()
{
  for (int i=0; i<rows; i++) delete ptr[i];
}
// --------------------------------------------------------
// operator dostepu do elementow macierzy
// --------------------------------------------------------
double& matrix::operator ()( int col,int row )
{
  return ptr[col][row];
}
// --------------------------------------------------------
