#include "./../../include/lib/nurbs.h"

#ifdef INCLUDE_SHEILD

#include "./../../include/lib/nurbs.h"
#include "./../../include/lib/matrix.h"

using namespace std;
template<class Pnt_>
void NurbsLibImp::NurbsCurve<Pnt_>::resize(int n, int Deg){
  deg_ = Deg ; 
  P_.resize(n) ;
  T_.resize(n+deg_+1) ; 
}

// Computes the non-zero basis functions into N of size deg+1
// The following relationship applies   N[i] <= N[span-deg+i]  for i = 0..deg
// A2.1 on p68 of the Nurbs Book
/*!
  \brief computes the non-zero basis functions of the curve

   Computes the non-zero basis functions and puts the result 
   into \a N. \a N has a size of deg+1. To relate \a N to the basis 
   functions, Basis[span -deg +i] = N[i] for i=0...deg.

   \latexonly
   The B-spline basis function of $p$-degree is defined as
   \begin{eqnarray}
   N_{i,0}(t) & = & \left\{ \begin{array}{ll} 1 & \mbox{if $t_i \leq t < t_{i+1}$} \\ 0 & \mbox{otherwise}\end{array}\right. \nonumber \\
   N_{i,p}(t) & = & \frac{t-t_i}{t_{i+p}-t_i}N_{i,p-1}(t)+\frac{t_{i+p+1}-t}{t_{i+p+1}-t_{i+1}}N_{i+1,p-1}(t) \nonumber
   \end{eqnarray}
   
   where the $t_i$ define the knot vector $T = \{t_0,\ldots,t_m\}$
   as a nondecreasing sequence of real numbers, {\em i.e.}, 
   $t_i \leq t_{i+1}$ for $i=0,\ldots,m-1$. And $m$ is related
   to the number of control points $n$ and the degree of the curve
   $p$ with the relation $m = n + p + 1$. The knot vector has
   the form
   
   \begin{equation}
   T=\{\underbrace{a,\ldots,a}_{p+1},t_{p+1},\ldots,t_{m-p-1},\underbrace{b,\ldots,b}_{p+1} \} 
   \end{equation}
   
   The B-spline basis function are non-zero for at most $p+1$ of
   the $N_{i,p}$. The relationship between the non-zero basis
   functions in N and $N_{i,p}$ is as follows, 
   $N_{span - deg + j, p} = N[j]$ for $j=0,\ldots,deg$. Where
   span is the non-zero span of the basis functions. This
   non-zero span for \a t can be found by calling 
   {\tt findSpan(t)}.

   \endlatexonly
   \htmlonly
    You can find more information in the LaTeX version.
   \endhtmlonly

   \param t  the parametric value
   \param i  the non-zero span of the basis functions
   \param N  the non-zero basis functions

   \warning \a t and \a i must be valid values
   \author Philippe Lavoie 
   \date 24 January 1997
*/

template<class Pnt_>
void NurbsLibImp::NurbsCurve<Pnt_>::basisFuns(const double t, const int i, vector<double>& N) const {
	double* left = (double*) alloca(2*(deg_+1)*sizeof(double)) ;
	double* right = &left[deg_+1] ;
	
	double temp,saved ;
	
	N.resize(deg_+1) ;
	
	N[0] = 1.0 ;
	for(size_t j=1; j<= deg_ ; j++){
		left[j] = t-T_[i+1-j] ;
		right[j] = T_[i+j]-t ;
		saved = 0.0 ;
		for(size_t r=0 ; r<j; r++){
			temp = N[r]/(right[r+1]+left[j-r]) ;
			N[r] = saved+right[r+1] * temp ;
			saved = left[j-r] * temp ;
			}
		N[j] = saved ;
		}	
}

/*!
  \brief Determines the knot span index

  Determines the knot span for which their exists non-zero basis 
  functions. The span is the index \a k for which the parameter 
  \a t is valid in the [t_k,t_{k+1}] range.
  
  \param t  the parametric value
  \return the span index at \a t.
  \warning \a t must be in a valid range
  \author Philippe Lavoie 
  \date 24 January 1997,
  \modified 20 January 1999 (Alejandro Frangi)
*/
template<class Pnt_>
int NurbsLibImp::NurbsCurve<Pnt_>::findSpan(double t) const {
	if(t>=T_[P_.size()]) 
		return P_.size()-1 ;
	if(t<=T_[deg_])
		return deg_ ;
	
	int low  = 0 ;
	int high = P_.size()+1 ; 
	int mid = (low+high)/2 ;
	
	while(t<T_[mid] || t>= T_[mid+1]){
		if(t<T_[mid])		high = mid ;
		else		low = mid ;
		mid = (low+high)/2 ;	}
		
//	cout<<"\nSPAN: "<<mid<<"\n";
	return mid ;	
}

#endif
#ifndef INCLUDE_SHEILD

void NurbsLibImp::knotAveraging(const vector<double>& tk, size_t degree, vector<double>& T){
	T.resize(tk.size()+degree+1) ;
	
	for(size_t j=1;j<tk.size()-degree;++j){
		T[j+degree] = 0.0 ;
		for(size_t i=j;i<j+degree;++i)
			T[j+degree] += tk[i] ;
		T[j+degree] /= (double)degree ;
		}
	for(size_t j=0;j<=degree;++j)	T[j] = 0.0 ;
	for(size_t j=T.size()-degree-1;j<T.size();++j) 	T[j] = tk.back() ;
}
/*
void knotAveraging(const vector<double>& tk, int degree, vector<double>& T){
	T.resize(tk.size()+degree+1) ;
	
	for(size_t j=1;j<tk.size()-degree;++j){
		T[j+degree] = 0.0 ;
		for(size_t i=j;i<j+degree;++i)
			T[j+degree] += tk[i] ;
		T[j+degree] /= (double)degree ;
		}
	for(int j=0;j<=degree;++j)	T[j] = 0.0 ;
	for(size_t j=T.size()-degree-1;j<T.size();++j) 	T[j] = 1.0 ;
}*/

#endif
#ifdef INCLUDE_SHEILD

template<class Pnt_>
double NurbsLibImp::chordLengthParam(const vector< Pnt_ >& Q, vector<double> &tb){
	double d(0);
	 
	tb.resize(Q.size()) ;
	tb[0] = 0 ; 
	for(size_t i=1;i<tb.size();i++)		d +=  ((Q[i].cvala()-Q[i-1].cvala()).apply(abs)).max()  ;	//oblicz dlugosc drogi (a walsciwie sume odleglosci miedzy punktami)
	for(size_t i=1;i<tb.size()-1;++i)		tb[i] = tb[i-1] + ((Q[i].cvala()-Q[i-1].cvala()).apply(abs).max() )/d ;	//ustaw czas proporcjonalnie do odleglosci (stala predkosc)
	tb[tb.size()-1] = 1.0 ; // In case there is some addition round-off
	return d ;
}

template<class Pnt_>
void NurbsLibImp::NurbsCurve<Pnt_>::globalInterp(const vector< Pnt_ >& Q, int d) {
	vector<double> tb ;

/*	//Usun punkty podwojne - wyliczona odleglosc tb miedzy tymi punktami bylaby rowna zero,
	// wiec mozna te punkty wyrzucic - uprosci to obliczenia
	vector< Point_nD<N_> > Q1(Q); 
	
	unique_copy(Q.begin(), Q.end(), back_inserter(Q1));	//kopiuje Q do Q1 c powtrzenia
	size_t k=0; // ilosc elementow usunietych
	for(size_t n=0; n<Q.size()-1; n++) {
		if (Q[n].v==Q[n+1].v) {
			Q1.erase(Q1.begin()+n-k);
			k++; }	}
*/			
	chordLengthParam(Q,tb) ; //oblicz tb (wektor czasu) proporcjonalnie do odleglosci miedzy punktami
	globalInterp(Q,tb,d) ; //tworz krzywa (punkty kotrolne)
	return;
}

/*!

  Global curve interpolation with points in nD and with the parametric values
  specified.

  \param Q  the nD points to interpolate
  \param tb  the parametric values 
  \param d  the degree of the interpolation

  \warning The number of points to interpolate must be greater than
               the degree specified for the curve.
*/
template<class Pnt_>
void NurbsLibImp::NurbsCurve<Pnt_>::globalInterp(const vector< Pnt_ >& Q, const vector<double>& tb, int d){

  if(d<=0){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else
//    Error err("globalInterp");
    cout << "The degree specified is equal or smaller than 0\n" ;
    cout << "deg = " << deg_ << endl ;
//    err.fatal() ;
#endif
  }
  if(d>=(int)Q.size()){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else 
//    Error err("globalInterp");
    cout << "The degree specified is greater then Q.n()+1\n" ;
    cout << "Q.n() = " << Q.size() << ", and deg = " << deg_ << endl ;
//    err.warning() ;
    d = Q.size()-1 ;  
#endif
  }

	resize(Q.size(),d) ;

//	double max=tb.back();
//	cout<<"MAX: "<<max<<"\n";
	
//	for (size_t n=0; n<tb.size(); n++)
//		tb[n]=tb[n]/max;
//	tb=tb/max;
	knotAveraging(tb,d,T_) ;
//	for (size_t n=0; n<tb.size(); n++)
//		tb[n]=tb[n]*max;
//	tb=tb*max;
//	cout<<"globalInterpolation: Znaleziony wektor wezlowy:\n"<<T;
//	cout<<"\nglobalInterpolation: Wektor tb:\n"<<tb;

	// Initialize the basis matrix A
			
	Matrix A(Q.size(),Q.size()) ;

	vector<double> N(deg_+1) ;
	
//	cout<<"Q.size:"<<Q.size()<<" tb: "<<tb;
	
	for(size_t i=1;i<Q.size()-1;i++){
		int span = findSpan(tb[i]);
//		cout<<"span("<<tb[i]<<"): "<<span<<"\n";
		basisFuns(tb[i],span,N) ;
//		cout<<"N: "<<N;
		for(size_t j=0;j<=deg_; j++)
			A(i, span-deg_+j) = (double)N[j] ;
	}

	A(0,0)  = 1.0 ;
	A(Q.size()-1,Q.size()-1) = 1.0 ;

 
  // Init matrix for LSE
  //http://www.ruhr-uni-bochum.de/num1/demo/numerics/NumericsUserGuide.htm
//  Matrix qq(d, Q.size()) ;
//  Matrix xx(d, Q.size()) ;
    // nazwa zmieniona na B_ ze wzgledu na konflikt z #define z player.h
	Matrix B_(Q.size(),Q[0].size()) ; //rozwiazuj od razu N_ rownan
	Matrix X_(Q.size(),Q[0].size()) ;


	for(size_t i=0;i<B_.rows();i++){
//		const double& rp = Q[i].v ; // this makes the SGI compiler happy
		for(size_t j=0; j<B_.cols(); j++) {
			B_(i,j) = Q[i].cvala()[j] ; //x[j] dla wielowym 
//			cout<<" j"<<j<<"i"<<i<<"Q[i...j]:"<<Q[i].cvala()[j];
		}	}
	
	solve(A,B_,X_) ;


//	cout<<"globalInterpolation: Macierz A:\n"<<A;
//	cout<<"globalInterpolation: Wektor (macierz) B:\n"<<B;
//	cout<<"globalInterpolation: Znalezione punkty kontrolne:\n"<<X;
//	A*=X;
//	cout<<"globalInterpolation: A*X:\n"<<A;
	
  // Store the data
	for(size_t i=0; i<X_.rows(); i++){ //P[i].x ma na razie tylko jeden wymiar
		for(size_t j=0; j<X_.cols(); j++)
			P_[i].vala()[j] = X_(i,j) ;
		P_[i].setweight(1.0) ;
	}
}
#if 0
/*!
  \brief It inserts a knot a number of times

  It inserts the knot t, r times and generates the curve nc.
  For more information, see A5.1 on page 151 of the NURBS book

  \param t  the knot to insert
  \param r  the number of times to insert \a t.
  \param nc  the resulting NURBS curve

  \return the number of knots inserted (0 if none)
  
  \warning the routine throws a NurbsError if the u value is not inside
           the clamped range.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class Pnt_>
int NurbsLibImp::NurbsCurve<Pnt_>::knotInsertion(double t, int r, NurbsCurve<Pnt_>& nc){
//cout<<"knotInsertion::Begin "<<"\n"<<flush;
 // Compute k and s      u = [ u_k , u_k+1)  with u_k having multiplicity s
  size_t k=0,s=0 ;
  size_t i,j ;
  int p = deg_ ;

  if(t<T_[deg_] || t>T_[P_.size()]){
#ifdef USE_EXCEPTION
    throw NurbsError();
#else
//    Error err("knotInsertion");
    cerr << "The parametric value isn't inside a valid range. " ; 
    cerr << "The valid range is between " << T_[deg_] << " and " << T_[P_.size()] << endl <<flush ;
//    err.fatal();
#endif
  }
   

  for(i=0;i<T_.size();i++){
    if(T_[i]>t) {
      k = i-1 ;
      break ;    }  }
//  vector<double>::const_iterator ;
 

  if(t<=T_[k]){
    s = 1 ;
    for(i=k;i>deg_;i--){
      if(T_[i]<=T_[i-1])
	s++ ;
      else
	break ;
    }
  }
  else{
    s=0 ;
  }
//cout<<"knotInsertion::s: "<<s<<"\n"<<flush;
  if(int(r+s)>p+1)
    r = p+1-s ;

  if(r<=0)
    return 0 ; 
///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//a co jesli nc wskazuje na this - tracimy dane

  nc.resize(P_.size()+r,deg_) ;

  // Load new knot vector
  for(i=0;i<=k;i++)    nc.T_[i] = T_[i] ;
  for(i=1;int(i)<=r;i++)    nc.T_[k+i] = t ;
  for(i=k+1;i<T_.size(); i++)    nc.T_[i+r] = T_[i] ;

  // Save unaltered control points
  vector< Point_nD<N_> > R_(p+1) ;

  for(i=0; i<=k-p ; i++)    nc.P_[i] = P_[i] ;
  for(i=k-s ; i< P_.size() ; i++)    nc.P_[i+r] = P_[i] ;
  for(i=0; i<=p-s; i++)    R_[i] = P_[k-p+i] ;

  // Insert the knot r times
  int L=0 ;
  double alpha ;
  for(j=1; (int)j<=r ; j++){
    L = k-p+j ;
    for(i=0;i<=p-j-s;i++){
      alpha = (t-T_[L+i])/(T_[i+k+1]-T_[L+i]) ;
      R_[i].vala() = alpha*R_[i+1].vala() + (1.0-alpha)*R_[i].vala() ;
    }
    nc.P_[L] = R_[0] ;
    if(p-j-s > 0)
      nc.P_[k+r-j-s] = R_[p-j-s] ;
  }

  // Load remaining control points
  for(i=L+1; i<k-s ; i++){
    nc.P_[i] = R_[i-L] ;
  }
  return r ; 
}
#endif

/*!
  \brief It inserts a knot a number of times

  It inserts the knot t, r times and generates the curve nc.
  For more information, see A5.1 on page 151 of the NURBS book

  \param t  the knot to insert
  \param r  the number of times to insert \a t.
  \param nc  the resulting NURBS curve

  \return the number of knots inserted (0 if none)
  
  \warning the routine throws a NurbsError if the u value is not inside
           the clamped range.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class Pnt_>
int NurbsLibImp::NurbsCurve<Pnt_>::knotInsertion(double t, int r){
//cout<<"knotInsertion::Begin "<<"\n"<<flush;
 // Compute k and s      u = [ u_k , u_k+1)  with u_k having multiplicity s
  size_t k=0,s=0 ;
  size_t i,j ;
  const int p = deg_ ;

  if(t<T_[deg_] || t>T_[P_.size()]){
#ifdef USE_EXCEPTION
    throw NurbsError();
#else
//    Error err("knotInsertion");
    cerr << "The parametric value isn't inside a valid range. " ; 
    cerr << "The valid range is between " << T_[deg_] << " and " << T_[P_.size()] << endl <<flush ;
//    err.fatal();
#endif
  }
   
	
	k=distance(T_.begin(), upper_bound(T_.begin()+p, T_.end()-p, t))-1;
	s=	k-(distance(T_.begin(), lower_bound(T_.begin()+p, T_.end()-p, t))-1);	

/*	//Ten kod tez dziala,
	if(t<=T_[k]){
		s = 1 ;
		for(i=k;i>deg_;i--){
			if(T_[i]<=T_[i-1])
			s++ ;
			else
			break ;}
	}
	else {
		s=0 ;	}*/
	
	cout<<"r+s: "<<r+s<<"   p+1: "<<p+1<<endl<<flush;
	if((int)(r+s)>p)
		r = p-s ;
	cout<<"r: "<<r<<endl<<flush;
	if(r<=0)		return 0 ; 

	// Save unaltered control points 
	vector< Pnt_ > R_(p+1) ;
	
//	for(i=0; i<=k-p ; i++)    nc.P_[i] = P_[i] ;
//	for(i=k-s ; i< P_.size() ; i++)    nc.P_[i+r] = P_[i] ;
	for(i=0; i<=p-s; i++)    R_[i] = P_[k-p+i] ;

	const Pnt_ pnt;
	cout<<"P_.size: "<<P_.size();
	P_.insert( P_.begin()+k-p+1, r, pnt);
	cout<<"  new: "<<P_.size()<<"  r:"<<r<<"\n"<<flush;
 
	// Insert the knot r times
	int L=0 ;
	double alpha ;
	for(j=1; (int)j<=r ; j++){
		L = k-p+j;
		for(i=0; i<=p-j-s; i++){
			alpha = (t-T_[L+i])/(T_[i+k+1]-T_[L+i]) ;
			cout<<"i+1:"<<i+1<<endl<<flush;
			cout<<"R_.size():"<<R_.size()<<endl<<flush;
			R_[i].vala() = alpha*R_[i+1].vala() + (1.0-alpha)*R_[i].vala() ;		}
	P_[L] = R_[0] ;
	if(p-j-s > 0)
		P_[k+r-j-s] = R_[p-j-s] ;
	}
	cout<<"Zyje jeszcze"<<endl<<flush;
  // Load new knot vector
/////  for(i=0;i<=k;i++)    nc.T_[i] = T_[i] ;
/////////  for(i=1;int(i)<=r;i++)    nc.T_[k+i] = t ;
////  for(i=k+1;i<T_.size(); i++)    nc.T_[i+r] = T_[i] ;
	T_.insert(T_.begin()+k+1, r, t);

  // Load remaining control points
	for(i=L+1; i<k-s ; i++){
		P_[i] = R_[i-L] ;  }
		
	return r ; 
}


/*!
  \brief Removes an internal knot from a curve.
  This is A5.8 on p185 from the NURB book modified to not check for 
  tolerance before removing the knot.

  \param nr  position of the knot to remove
  \param num  the number of times to try to remove the knot

  \warning r \b must be an internal knot.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class Pnt_>
void NurbsLibImp::NurbsCurve<Pnt_>::removeKnot(int  nr, int num)
{
  const int s = count(T_.begin(), T_.end(), T_[nr]);  // s  the multiplicity of the knot
 
//  cout<< "Wielokrotnosc wezla s = "<<s<<"\n";
//  cout<< "Numer i wartosc = "<<nr<<", "<<T_[nr]<<"\n";
  const int m = T_.size() ;
  const int ord = deg_+1 ;
  const int fout = (2*nr-s-deg_)/2 ;
  /*const*/ int last = nr-s ;
  /*const*/ int first = nr-deg_ ;
  
  double alfi, alfj ;
  int i,j,k,ii,jj,off ;
  double t ;

  vector< Pnt_ > temp( 2*deg_+1 ) ;

  t = T_[nr] ;

  if(num<1){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else   
//    Error err("removeKnot");
    cerr << "A knot can only be removed a positive number of times!\n" ;
    cerr << "num = " << num << endl ;
//    err.fatal() ;
#endif
  }

  int ts;
  for(ts=0; ts<num; ++ts){
    off = first-1 ;
    temp[0] = P_[off] ;
    temp[last+1-off] = P_[last+1] ;
    i = first; j = last ;
    ii = 1 ; jj = last-off ;
    while(j-i > ts){
      alfi = (t-T_[i])/(T_[i+ord+ts]-T_[i]) ;
      alfj = (t-T_[j-ts])/(T_[j+ord]-T_[j-ts]) ;
      temp[ii].vala() = (P_[i].vala()-(1.0-alfi)*(temp[ii-1]).vala())/alfi ;
      temp[jj].vala() = (P_[j].vala()-alfj*temp[jj+1].vala())/(1.0-alfj) ;
      ++i ; ++ii ;
      --j ; --jj ;
    }
    i = first ; j = last ;
    while(j-i>ts){
      P_[i].vala() = temp[i-off].vala() ;
      P_[j].vala() = temp[j-off].vala() ;
      ++i; --j ;
    }
    --first ; ++last ;
  }
  if(ts==0) {
#ifdef USE_EXCEPTION
    throw NurbsError();
#endif
    cerr << "Major error happening... t==0\n" ;
    return ;
  }

  for(k=nr+1; k<m ; ++k)
    T_[k-ts] = T_[k] ;
  j = fout ; i=j ; // Pj thru Pi will be overwritten
  for(k=1; k<ts; k++)
    if( (k%2) == 1)
      ++i ;
    else
      --j ;
  for(k=i+1; size_t(k) < P_.size() ; k++) {// Shift
    P_[j++] = P_[k] ; 
  }

  resize(P_.size()-ts,deg_) ;

  return ;
}


#endif
