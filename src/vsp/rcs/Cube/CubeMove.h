/////////////////////////////////////////////////////////
// Nazwa: CubeMove.h
// Opis: Klasa zawierajaca wszystkie stale zwiazane z ruchem 
//      wykonywanym na kostce Rubika.
/////////////////////////////////////////////////////////

#ifndef CubeMove_h
#define CubeMove_h

#include "CubeException.h"

class CubeMove
{
  public:
         
    enum eMove
    {
        F, R, U, B, L, D
    };    

    const static int MOVE_NUMBER = D + 1;

    // nazwy obrotow
    const static char MOVE_NAMES[MOVE_NUMBER+1];

    enum eTurn
    {
        QUARTER, HALF, ANTI_QUARTER
    };
    const static int TURN_NUMBER = ANTI_QUARTER + 1;
    
    // nazwy krotnosci
    const static char TURN_NAMES[TURN_NUMBER+1];

    static eMove MOVE_NUMBER_FROM_NAME(char name)
        throw (CubeException);
    static eTurn TURN_NUMBER_FROM_NAME(char name)
        throw (CubeException);
    
    static eMove OPPOSITE_FACE_MOVE(eMove move);
    static eTurn REVERSE_TURN(eTurn turn);
         
};   

#endif // CubeMove_h
