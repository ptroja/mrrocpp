#ifndef _nurbs_h
#define _nurbs_h

#include <iostream>
#include <valarray> //arytmetyka wektorowa
#include <vector>
#include <cmath>
#include <math.h>

#include "matrix.h"
#include "point_nD.h"
	
namespace NurbsLibImp {
using namespace std;

//template<class Pnt_ >
class PointPtr {
private:
	Point* ptr_;
public:
	explicit PointPtr(Point* ptr): ptr_(ptr) {}
	Point* operator->() const {return ptr_;}
	Point* get() const {return ptr_;}
	Point& operator*() const {return *ptr_;} 
};

class NurbsCurveAbstract {
public:
	virtual ~NurbsCurveAbstract() { };

	virtual const Point& curvePoint(const double time_param)  =0;
	
	virtual size_t size() const =0;
	virtual void resize(int n, int deg) =0;
	virtual void reserve(int n) =0;
	virtual int capacity() const =0;
	
	virtual double maxT() const =0;
	virtual double minT() const =0;
	virtual size_t& degree() =0;
	
	virtual void changeKnot(size_t i, double value) =0;
	virtual void changePoint(size_t i, const Point&) =0;
};

template<class Pnt_ > class NurbsCurve: public NurbsCurveAbstract {
private:
	vector< Pnt_ > P_; // the vector of control points
	vector<double> T_ ;  // the knot vector 
	size_t deg_ ;  // the degree of the NURBS curve
	Pnt_ TP_;  //tempolary point to store curvePoint() result data.
protected:
	void basisFuns(const double t, const int i, vector<double>& N) const;
	int findSpan(double t) const ;	
public:
	NurbsCurve(): P_(0, Pnt_()),T_(0),deg_(0), TP_() { }
	NurbsCurve(size_t n, size_t deg): P_(n, Pnt_() ), T_(n+deg+1), deg_(deg), TP_() { }
	NurbsCurve(const vector< Pnt_ >& P, const vector<double>& T, int deg): P_(P),T_(T),deg_(deg), TP_() { }

	void resize(int n, int Deg);
	void globalInterp(const vector< Pnt_ >& Q, int d);
	void globalInterp(const vector< Pnt_ >& Q, const vector<double>& tb, int d);
	
	size_t size() const {return P_.size();};
	void reserve(int n) {P_.reserve(n); T_.reserve(n+deg_+1); }
	int capacity() const {return  (P_.capacity()>T_.capacity()) ? P_.capacity() : T_.capacity(); }
	
	double maxT() const {return T_.back();}
	double minT() const {return T_.front();}
	vector<double>& knotVector() {return T_;}
	double& knotVector(size_t i) {return T_[i];}
	vector< Pnt_ >& pointsVector() {return P_;}
	Pnt_& pointsVector(size_t i) {return P_[i];}
	size_t& degree() {return deg_;}
	void PCP(int i) { cout<<"\nCP: "<<P_[i].vala();};
	void PkV() { cout<<"\nknotV: "<<T_;};
	
	void changeKnot(size_t i, double value) {T_[i]=value;};
	void changePoint(size_t i, const Point& P) {P_[i].vala()=P.cvala(); };

//	Point* curvePoint(const double time_param, Point* pnt) const {return pnt;};
	const Point& curvePoint(const double t)  {// zwraca punkt na krzywej N_ wymiarowej
		vector<double> N;
		const int span = findSpan(t);
		basisFuns(t, span, N);
//		Pnt_ point;
		TP_.vala() = 0.0;
		for(size_t i=0; i<=deg_; i++) {
		 	TP_.vala()[Pnt_::mask]+=(N[i]*P_[span-deg_+i].weight()*P_[span-deg_+i].vala() )[Pnt_::mask]; 
		 	TP_.setweight(TP_.weight()+N[i]*P_[span-deg_+i].weight()); }
		TP_.vala()/=TP_.weight();
//		cout<<"\nP:   "<<P[span-deg].vala(); 
		return TP_;}
		
	int knotInsertion(double t, int r, NurbsCurve< Pnt_ >& nc);
	int knotInsertion(double t, int r);
	void removeKnot(int  n, int r);
};

void knotAveraging(const vector<double>&, size_t , vector<double>& );
template <class Pnt_> double chordLengthParam(const vector< Pnt_ >& Q, vector<double> &tb);
//}
} //namespace NurbsLib

#define INCLUDE_SHEILD
#include "./../../src/lib/nurbs.cc"
#undef INCLUDE_SHEILD

namespace NurbsLib {
	using NurbsLibImp::NurbsCurveAbstract;
	using NurbsLibImp::NurbsCurve;
	using NurbsLibImp::Point_nD;
	using NurbsLibImp::Point; }
	
#endif
