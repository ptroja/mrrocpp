/////////////////////////////////////////////////////////
// Nazwa: CubeTester.h
// Opis: Klasa CubeTester testow ogolnych klas kostki.
/////////////////////////////////////////////////////////

#ifndef CubeTester_h
#define CubeTester_h

#include "../General/Tester.h"
#include "CubeMove.h"


class Cube;
class CubeException;

// Klasa wyjatkow operacji kombinatorycznych.
class CubeTester : public Tester
{

  public:
         
    CubeTester() {}
    virtual ~CubeTester() {}
         
    virtual bool Test();
    
    
  private:

    bool TestFaceletCube();
    bool TestCubieCube();
    bool TestTranslator();
    
    bool TestFaceletCubeStringException(const char* scube);
    bool TestFaceletCubeTableException(const int icube[]);
    bool TestFaceletCubeMoveException();
    bool TestFaceletCubeCreate();
    bool TestFaceletCubeString(const char* scube);
    bool TestFaceletCubeTables();

    bool TestCubieCubeTableException(const int icube[]);
    bool TestCubieCubeCreate();
    bool TestCubieCubeTables();
    bool TestCubieCubeMoves();

    bool TestTranslatorConst();
    bool TestTranslatorTranslateException(const char* scube);
    bool TestTranslatorTranslate();

    void GetMoves(CubeMove::eMove *moves, CubeMove::eTurn *turns, Cube *ref, int number);
    void Move(Cube *cube, CubeMove::eMove *moves, CubeMove::eTurn *turns, int number, bool forward);
    void Print(CubeMove::eMove *moves, CubeMove::eTurn *turns, int number);
    void Print(const char *desc, Cube *cube);
    void Print(const char *desc, CubeException *exp);
};


#endif // CubeTester_h
