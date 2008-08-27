/*
 * ecp_matrix4x4.h
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#ifndef ECP_MATRIX4X4_H_
#define ECP_MATRIX4X4_H_

#define TYPE_T 1
#define TYPE_D 2
#define TYPE_S 0

#define A_LOADED 1
#define B_LOADED 2

class Matrix4x4
{
    double A[16], b[4], x[4], inv[16];
    bool solved;
    char full;
    short type;

  public:
    Matrix4x4();
    Matrix4x4(double *, short=TYPE_S);
    Matrix4x4(double *, double *, short=TYPE_S);

    double * getA();
    double * getb();
    double * getx();
    double * getinv();

    void setA(double *, short=TYPE_S);
    void setb(double *);

    double det3x3(short x, short y);
    double det4x4();
    void adj4x4(double * out);
    void inv_matrix4x4();
    void solveAxb4x4();

    void rproduct4x4(double *);
    void lproduct4x4(double *);
    void product4x1(double *);
    void product1x4(double *);
};


#endif /* ECP_MATRIX4X4_H_ */
