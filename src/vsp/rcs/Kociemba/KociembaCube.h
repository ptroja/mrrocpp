/////////////////////////////////////////////////////////
// Nazwa: KociembaCube.h
// Opis: Klasa KociembaCube zawierajaca opis kostki Rubika
//      na potrzeby algorytmu Kociemby.
/////////////////////////////////////////////////////////

#ifndef KociembaCube_h
#define KociembaCube_h

#include "../Cube/Cube.h"
#include "../Cube/CubieCube.h"
#include "KociembaException.h"


// Klasa opisujaca Kostke Rubika na poziomie wspolrzednych.
class KociembaCube : public Cube 
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }

    
  public:
    const static int EDGE_SLICE_NUMBER = CubieCube::EDGE_NUMBER / 3;

    const static int CORNER_PERMUTATION_NUMBER;  
    const static int CORNER_ORIENTATION_NUMBER;  
    const static int EDGE_SLICE_COMBINATION_NUMBER;  
    const static int EDGE_SLICE_PERMUTATION_NUMBER;  
    const static int EDGE_TWO_SLICES_PERMUTATION_NUMBER;  
    const static int EDGE_ORIENTATION_NUMBER;  

  protected:
         
    // Ustawia wspolrzedne stanu poczatkowego.
    static void InitCoords(int coords[]);

    // Pobiera stan kostki z tabeli reprezentujacej go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    static void CubieCubeToCoords(CubieCube& cube, int coords[]);

    // Zapisuje stan kostki do tabeli
    static void CoordsToCubieCube(CubieCube& cube, int coords[])
           throw (CubeException);


  public:

    // Pobiera stan kostki z tabeli reprezentujacej go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    virtual void FromCubieCube(CubieCube& cube)
            throw (KociembaException) = 0;

    // Zapisuje stan kostki do tabeli
    virtual void ToCubieCube(CubieCube& cube) = 0;

    // Pobiera stan kostki ze wspolrzednych reprezentujacych go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    virtual void FromCoords(int coords[]) = 0;

    // Zapisuje stan kostki do wspolrzednych
    virtual void ToCoords(int coords[]) = 0;

};


#endif // KociembaCube_h
