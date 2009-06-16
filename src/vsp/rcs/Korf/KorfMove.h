/////////////////////////////////////////////////////////
// Nazwa: KorfMove.h
// Opis: Klasa KorfMove zawierajaca dane o ruchach na kostce
//       Korfa.
/////////////////////////////////////////////////////////

#ifndef KorfMove_h
#define KorfMove_h

#include "../Cube/Cube.h"
#include "../Cube/CubeMove.h"
#include "../Cube/CubieCube.h"


class KorfMoveTable;

// Klasa zawierajace dane o ruchach na kostce.
class KorfMove
{
  private:
    static bool bInitialized;
    static KorfMoveTable *mtCP, *mtCO, *mtE1CP, *mtE1CO, *mtE2CP, *mtE2CO;
    
  public:
    static void Init();
    void MoveCoords(int coords[], CubeMove::eMove move);
   
};


#endif // KorfMove_h

