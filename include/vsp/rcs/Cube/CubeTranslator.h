/////////////////////////////////////////////////////////
// Nazwa: CubeTranslator.cpp
// Opis: Klasa CubeTranslator tlumaczaca klasy reprezentujace kostke.
/////////////////////////////////////////////////////////

#ifndef CubeTranslator_h
#define CubeTranslator_h

#include "CubieCube.h"

class FaceletCube;

// Klasa opisujaca kostke Rubika na poziomie scianek kostki. Zawiera ona
// zawartosc 54 scian kostki. Umozliwia sprawdzanie poprawnosci stanu kostki.
class CubeTranslator
{
  public:
    // Numery scianek nalezacych do rogow.
    const static int CORNER_MAP[CubieCube::CORNER_NUMBER][CubieCube::TWIST_NUMBER];
    // Numery scianek nalezacych do krawedzi.
    const static int EDGE_MAP[CubieCube::EDGE_NUMBER][CubieCube::FLIP_NUMBER];

  public:
    static void FaceletToCubie(FaceletCube& fCube, CubieCube& cCube);
    static void CubieToFacelet(FaceletCube& fCube, CubieCube& cCube);
};

#endif // CubeTranslator_h
