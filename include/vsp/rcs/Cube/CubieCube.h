/////////////////////////////////////////////////////////
// Nazwa: CubieCube.cpp
// Opis: Klasa CubieCube opisujaca kostke na kosteczek.
/////////////////////////////////////////////////////////

#ifndef CubieCube_h
#define CubieCube_h

#include "Cube.h"
#include "FaceletCube.h"
#include "CubeMove.h"
#include "CubeException.h"
#include "../Combinatorials/Permutation.h"
#include "../Combinatorials/Variation.h"


// Klasa opisujaca kostke Rubika na poziomie kosteczek. Zawiera ona permutacje
// i orientacje 12 krawedzi i 8 rogow kostki. Umozliwia sprawdzanie poprawnosci 
// stanu kostki.
class CubieCube : public Cube
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }

    
  public:
         
    // Liczba rogow.
    const static int CORNER_NUMBER = 8;

    // Numery rogow.
    enum eCorner
    {
        URF, UFL, ULB, UBR,   // 0 - 3   rogi z gornej warstwy
        DFR, DLF, DBL, DRB    // 4 - 7   rogi z dolnej warstwy
    };
    
    enum eTwist
    {
        NO_QUARK, // 0
        QUARK, ANTI_QUARK, // 1, 2
        CORNER_TWIST_NUMBER // 3
    };    
    const static int TWIST_NUMBER = 3;

    // Liczba rogow w scianie.
    const static int CORNER_IN_FACE_NUMBER = 4;

    // Rogi nalezace do scian kostki
    const static int MOVE_CORNERS[CubeMove::MOVE_NUMBER][CORNER_IN_FACE_NUMBER];

    // Czy ruch obraca rogi nalezace do scian kostki
    const static bool MOVE_TWIST[CubeMove::MOVE_NUMBER];

    // Obrocenie rogu
    static void CORNER_TWIST(int& ori, bool clockwise);

    // Rotacja rogow w scianie.
    static void FACE_CORNERS_ROTATE(FaceletCube::eFace face, bool clockwise);


    // Liczba krawedzi.
    const static int EDGE_NUMBER = 12;

    // Numery krawedzi.
    enum eEdge
    {
        UR, UF, UL, UB, // 0 - 3    krawedzie z gornej warstwy
        DR, DF, DL, DB, // 4 - 7    krawedzie z dolnej warstwy
        FR, FL, BL, BR  // 8 - 11   krawedzie ze srodkowej warstwy
    };

    enum eFlip
    {
        NOT_FLIPPED, // 0
        FLIPPED, // 1
        EDGE_FLIP_NUMBER // 2
    };    
    const static int FLIP_NUMBER = 2;

    // Liczba rogow w scianie.
    const static int EDGE_IN_FACE_NUMBER = 4;

    // Krawedzi nalezace do scian kostki
    const static int MOVE_EDGES[CubeMove::MOVE_NUMBER][EDGE_IN_FACE_NUMBER];

    // Czy ruch obraca krawedzi nalezace do scian kostki
    const static bool MOVE_FLIP[CubeMove::MOVE_NUMBER];

    // Obrocenie rogu
    static void EDGE_FLIP(int& ori);

    // Rotacja krawedzi w scianie.
    static void FACE_EDGES_ROTATE(FaceletCube::eFace face, bool clockwise);

    
  private:
    
    // Permutacja rogow kostki prezentowana jako tabela zawierajaca numery
    // rogow w kolejnosci zdefiniowanej przez stale w klasie CubeConst.
    Permutation cornerPermutation;
    
    // Orientacja rogow kostki prezentowana jako tabela zawierajaca numery
    // orientacji rogow w kolejnosci zdefiniowanej przez stale w klasie
    // CubeConst.
    Variation cornerOrientation;
    
    // Permutacja krawedzi kostki prezentowana jako tabela zawierajaca numery
    // krawedzi w kolejnosci zdefiniowanej przez stale w klasie CubeConst.
    Permutation edgePermutation;
    
    // Orientacja krawedzi kostki prezentowana jako tabela zawierajaca numery
    // orientacji odpowiednich krawedzi w kolejnosci zdefiniowanej przez stale
    // w klasie CubeConst.
    Variation edgeOrientation;
   
    
  public:
         
    // Konstruktor domyslny. Tworzy ulozona kostke.
    CubieCube();
    
    // Konstruktor kopiujacy. Tworzy nowy stan rowny stanowi cube.
    CubieCube(const CubieCube& cube);

    // Kopiuje stan kostki. Zwraca nowy stan, dla ktore alokuje pamiec.
    virtual Cube* Clone() { return new CubieCube(*this); }

    // Destruktor.
    ~CubieCube();
    
    // Operator porownia. Porownuje ze stanem kostki cube.
    int operator==(const CubieCube& cube);
    
    // Ustawia stan kostki jako ulozony. Nie zwraca zadnej wartosci.
    void SetInitState();


    // Pobiera stan kostki z tabeli reprezentujacej go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    void FromTable(const int cube[]) 
         throw (CubeException);

    // Zapisuje stan kostki do tabeli
    void ToTable(int cube[]);

    // Zamienia stan kostki na lancuch znakow reprezentujacy go. Zwraca tenze
    // lancuch znakow. Alokuje pamiec na lancuch, ktora trzeba pozniej zwolnic.
    char* ToString();


    // Wykonuje ruch o numerze move na kostce. Nie zwraca zadnej wartosci.
    void Move(CubeMove::eMove move, CubeMove::eTurn turn);
    
    
  private:
    
    // Sprawdza poprawnosc stanu kostki. Orientacja rogow i krawedzi powinna
    // byc parzysta. Dodatkowo parzystosc permutacji rogow i krawedzi powinny
    // byc sobie rowne.
    void Validate() throw (CubeException);


    // Wykonuje obrot move zgodnie z badz przeciwnie do (w zaleznosci od 
    // wartosci parametry clockwise) ruchu wskazowek zegara.
    void MoveCubies(CubeMove::eMove move, bool clockwise);

};

#endif // CubieCube_h    
