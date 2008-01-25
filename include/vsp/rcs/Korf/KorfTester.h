/////////////////////////////////////////////////////////
// Nazwa: KorfTester.h
// Opis: Klasa KorfTester testow ogolnych klas kostki.
/////////////////////////////////////////////////////////

#ifndef KorfTester_h
#define KorfTester_h

#include "../General/Tester.h"
#include "../Cube/CubeMove.h"
#include "../Cube/CubeSolution.h"
#include "KorfMoveTable.h"
#include "KorfMoveTable_CP.h"
#include "KorfMoveTable_CO.h"
#include "KorfMoveTable_ECP.h"
#include "KorfMoveTable_ECO.h"


class Cube;
class CubeException;

// Klasa wyjatkow operacji kombinatorycznych.
class KorfTester : public Tester
{
      
  public:
         
    KorfTester() {}
    virtual ~KorfTester() {}
         
    virtual bool Test();
    
    
  private:

    bool TestKorfCube();
    bool TestKorfSolver();

    bool TestKorfCubeCreate();
    bool TestKorfCubeCubie();
    bool TestKorfCubeMoveTable(KorfMoveTable* mt);
    bool TestKorfCubeMove();
    
    bool TestKorfSolverSolve();


    void GetMoves(CubeMove::eMove *moves, CubeMove::eTurn *turns, Cube *ref, int number);
    void Move(Cube *cube, CubeMove::eMove *moves, CubeMove::eTurn *turns, int number, bool forward);
    void Print(CubeMove::eMove *moves, CubeMove::eTurn *turns, int number);
    void Print(const char *desc, Cube *cube);
    void Print(const char *desc, CubeSolution *sol);
    //void Print(const char *desc, CubeException *exp);

};


#endif // KorfTester_h
