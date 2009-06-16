/////////////////////////////////////////////////////////
// Nazwa: KociembaTester.h
// Opis: Klasa KociembaTester testow ogolnych klas kostki.
/////////////////////////////////////////////////////////

#ifndef KociembaTester_h
#define KociembaTester_h

#include "../General/Tester.h"
#include "../Cube/CubeMove.h"
#include "KociembaSolution.h"
#include "KociembaMoveTable.h"
#include "KociembaPhase1Cube.h"


class Cube;
class CubeException;

// Klasa wyjatkow operacji kombinatorycznych.
class KociembaTester : public Tester
{
      
  public:
         
    KociembaTester() {}
    virtual ~KociembaTester() {}
         
    virtual bool Test();
    
    
  private:

    bool TestKociemba1Cube();
    bool TestKociemba2Cube();
    bool TestKociembaSolver();

    bool TestKociemba1CubeCreate();
    bool TestKociemba1CubeCubie();
    bool TestKociemba1CubeMoveTable(KociembaMoveTable* mt);
    bool TestKociemba1CubeMove();
    
    bool TestKociemba2CubeCubieException(CubieCube& ccube);
    bool TestKociemba2CubeCreate();
    bool TestKociemba2CubeCubie();
    bool TestKociemba2CubeMoveTable(KociembaMoveTable* mt);
    bool TestKociemba2CubeMoveException(CubeMove::eMove move, CubeMove::eTurn turn);
    bool TestKociemba2CubeMove();
    
    bool TestKociembaSolverSolvePhase1();
    bool TestKociembaSolverSolvePhase1Next(KociembaPhase1Cube& cube, CubeSolution *sol);
    bool TestKociembaSolverSolvePhase2Exception(CubieCube& ccube);
    bool TestKociembaSolverSolvePhase2();
    bool TestKociembaSolverSolve();
    bool TestKociembaSolverSolveNext(CubieCube& cube, KociembaSolution *sol);
    bool TestKociembaSolverSolveOptimal();


    void GetMoves(CubeMove::eMove *moves, CubeMove::eTurn *turns, Cube *ref, int number);
    void Move(Cube *cube, CubeMove::eMove *moves, CubeMove::eTurn *turns, int number, bool forward);
    void Print(CubeMove::eMove *moves, CubeMove::eTurn *turns, int number);
    void Print(const char *desc, Cube *cube);
    void Print(const char *desc, CubeSolution *sol);
    void Print(const char *desc, KociembaException *exp);

};


#endif // KociembaTester_h
