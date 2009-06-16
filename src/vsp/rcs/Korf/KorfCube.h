/////////////////////////////////////////////////////////
// Nazwa: KorfCube.h
// Opis: Klasa KorfCube zawierajaca opis kostki Rubika
//      na potrzeby algorytmu Korfa.
/////////////////////////////////////////////////////////

#ifndef KorfCube_h
#define KorfCube_h

#include "../Cube/Cube.h"
#include "../Cube/CubeMove.h"
#include "../Cube/CubieCube.h"


class KorfMoveTable;

// Klasa opisujaca Kostke Rubika na poziomie wspolrzednych.
class KorfCube : public Cube 
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }

    
  public:
    const static int EDGE_HALF_NUMBER = CubieCube::EDGE_NUMBER / 2;

  public:
    const static int CORNER_PERMUTATION_NUMBER;  
    const static int CORNER_ORIENTATION_NUMBER;  
    const static int EDGE_HALF_COMBINATION_NUMBER;  
    const static int EDGE_HALF_PERMUTATION_NUMBER;  
    const static int EDGE_HALF_ORIENTATION_NUMBER;  
    const static int CORNER_COORD_NUMBER;  
    const static int EDGE_HALF_COORD_NUMBER;  
      
  public:
    static int CORNER(int perm,int orie)    { return (orie*CORNER_PERMUTATION_NUMBER+perm); }
    static int EDGE(int comb,int perm,int orie) { return ((comb*EDGE_HALF_ORIENTATION_NUMBER+orie)*EDGE_HALF_PERMUTATION_NUMBER+perm); }
    static int CORNER_PERMUTATION(int corner) { return (corner%CORNER_PERMUTATION_NUMBER); }
    static int CORNER_ORIENTATION(int corner) { return (corner/CORNER_PERMUTATION_NUMBER); }
    static int EDGE_COMBINATION(int edge) { return (edge/(EDGE_HALF_PERMUTATION_NUMBER*EDGE_HALF_ORIENTATION_NUMBER)); }
    static int EDGE_PERMUTATION(int edge) { return (edge%EDGE_HALF_PERMUTATION_NUMBER); }
    static int EDGE_ORIENTATION(int edge) { return (edge/EDGE_HALF_PERMUTATION_NUMBER%EDGE_HALF_ORIENTATION_NUMBER); }

    //static int EDGE(int comb,int perm,int orie) { return ((orie*EDGE_HALF_PERMUTATION_NUMBER+perm)*EDGE_HALF_COMBINATION_NUMBER+comb); }
    //static int EDGE_COMBINATION(int edge) { return (edge%EDGE_HALF_COMBINATION_NUMBER); }
    //static int EDGE_PERMUTATION(int edge) { return (edge/EDGE_HALF_COMBINATION_NUMBER%EDGE_HALF_PERMUTATION_NUMBER); }
    //static int EDGE_ORIENTATION(int edge) { return (edge/EDGE_HALF_COMBINATION_NUMBER/EDGE_HALF_PERMUTATION_NUMBER); }
         
  private:
    int cornerCoord;
    int edgeFirstHalfCoord;
    int edgeSecondHalfCoord;

  private:
    static bool bInitialized;
    static KorfMoveTable *mtCP, *mtCO, *mtE1CP, *mtE1CO, *mtE2CP, *mtE2CO;
    
  public:
    static void SInit();
    static void SClear();

  public:
         
    // Konstruktor domyslny. Tworzy ulozona kostke.
    KorfCube();
    
    // Konstruktor kopiujacy. Tworzy nowy stan rowny stanowi cube.
    KorfCube(const KorfCube& cube);

    // Destruktor.
    ~KorfCube();
    
    // Kopiuje stan kostki. Zwraca nowy stan, dla ktore alokuje pamiec.
    virtual Cube* Clone() { return new KorfCube(*this); }
    
    // Operator porownia. Porownuje ze stanem kostki cube.
    int operator==(const KorfCube& cube);
    
    // Ustawia stan kostki jako ulozony. Nie zwraca zadnej wartosci.
    void SetInitState();


    // Pobiera stan kostki z tabeli reprezentujacej go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    void FromCubieCube(CubieCube& cube);

    // Zapisuje stan kostki do tabeli
    void ToCubieCube(CubieCube& cube);

    // Pobiera stan kostki ze wspolrzednych reprezentujacych go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    void FromCoords(int coords[]);

    // Zapisuje stan kostki do wspolrzednych
    void ToCoords(int coords[]);


    // Zamienia stan kostki na lancuch znakow reprezentujacy go. Zwraca tenze
    // lancuch znakow. Alokuje pamiec na lancuch, ktora trzeba pozniej zwolnic.
    char* ToString();
   

    // Wykonuje ruch o numerze move na kostce. Nie zwraca zadnej wartosci.
    virtual void Move(CubeMove::eMove move, CubeMove::eTurn turn);

  private:
    void MoveCoords(CubeMove::eMove move);
    
    
  public:  
    int static MoveCoord(int coordNum, int coordValue, CubeMove::eMove move);
   
};


#endif // KorfCube_h

