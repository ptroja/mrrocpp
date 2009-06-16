#ifndef _point_nD_h
#define _point_nD_h

#include <valarray> //arytmetyka wektorowa

namespace NurbsLibImp {
using namespace std;
template<size_t N_> class Mask {
protected:
	valarray<bool> m_;
public:
	Mask():m_(true,N_+1) {m_[N_]=false;}
	const valarray<bool> & const_ref() const {return m_;}
};

class Point {
public:
//	Point() {;}; //Konstruktor powinien by generowany automatycznie - ale nie jest, wiec ta linijka musi tu byc
	virtual size_t size() const =0;
	virtual const double weight() const =0;
	virtual const double setweight(double new_weight) =0; 
	virtual valarray<double> & 	vala() =0;
	virtual const valarray<double> & cvala() const =0;

	virtual double operator[](size_t) const =0;
	virtual double& operator[](size_t) =0;
	static const valarray<bool>& mask;

	virtual ~Point() {;};
};

template<size_t N_> class Point_nD: public Point{
protected:
	valarray<double> v_; //n+1-dimensional vector; last dimension - weight
	static const Mask<N_> m_;
public:
	static const valarray<bool>& mask; 
//	class NurbsError: public exception {};
	Point_nD(): v_(0.0, N_+1)  {v_[N_]=1.0; }//ostatni element wektora v_ przechowuje wage
	Point_nD(const valarray<double>& v): v_(v) {}
	
	size_t size() const {return v_.size();}
	valarray<double> & 	vala() {return v_;} 
	const valarray<double> & cvala() const {return v_;}
 
	const double weight() const {return v_[N_];} //nie zwracac referencji do elementow valarray - po zwroceniu moga byc niepoprawne
	const double setweight(double w) {v_[N_]=w; return v_[N_];}

	double operator[](size_t i) const {return v_[i];}
	double& operator[](size_t i) {return v_[i];}
//	const double cweight() const {return v_[N_];}

//	operator const valarray<double>&() const {return v_;}//valarray<double>(v_);}
//	Point_nD& operator=(valarray<double>& v) {v_=v; return *this;}

	Point_nD & operator+=(const Point_nD & p) {v_+=p.v_; return *this; } 
	Point_nD & operator*=(const Point_nD & p) {v_*=p.v_; return *this; }
	Point_nD & operator-=(const Point_nD & p) {v_-=p.v_; return *this; }
	Point_nD & operator/=(const Point_nD & p) {v_/=p.v_; return *this; }
//	Point_nD & operator+=(const valarray<double>& v) {v_[mask]+=v; return *this; } 
//	Point_nD & operator*=(const valarray<double>& v) {v_[mask]*=v; return *this; } 
//	Point_nD & operator-=(const valarray<double>& v) {v_[mask]-=v; return *this; } 
//	Point_nD & operator/=(const valarray<double>& v) {v_[mask]/=v; return *this; } 

};

template<size_t N_>	
const Mask<N_> Point_nD<N_>::m_=Mask<N_>();

template<size_t N_>	
const valarray<bool>& Point_nD<N_ >::mask=m_.const_ref()  ;

} //namespace


#endif
