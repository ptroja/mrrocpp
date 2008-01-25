/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase1Cube.h
// Opis: Klasa KociembaPhase1Cube zawierajaca opis kostki Rubika
//      na potrzeby algorytmu Kociemby.
/////////////////////////////////////////////////////////

#ifndef KociembaPhase1Cube_h
#define KociembaPhase1Cube_h

#include "KociembaCube.h"
#include "KociembaException.h"


class KociembaPhase1MoveTable;

// Klasa opisujaca Kostke Rubika na poziomie wspolrzednych.
class KociembaPhase1Cube : public KociembaCube 
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }

    
  private:
    int cornerOrientationCoord;
    int edgeMidSliceCombinationCoord;
    int edgeOrientationCoord;
    
  private:
    static bool bInitialized;

    // Tabela przeksztalcen wspolrzednych orientacji rogow przez obroty
    // uzywana w pierwszej fazie algorytmu
    static KociembaPhase1MoveTable *mtCO;

    // Tabela przeksztalcen wspolrzednych kombinacji pozycji krawedzi srodkowej
    // warstwy przez obroty uzywana w pierwszej fazie algorytmu
    static KociembaPhase1MoveTable *mtEMC;

    // Tabela przeksztalcen wspolrzednych orientacji krawedzi przez obroty
    // uzywana w pierwszej fazie algorytmu
    static KociembaPhase1MoveTable *mtEO;
    
  public:
    static void SInit();
    static void SClear();


  public:
         
    // Konstruktor domyslny. Tworzy ulozona kostke.
    KociembaPhase1Cube();
    
    // Konstruktor kopiujacy. Tworzy nowy stan rowny stanowi cube.
    KociembaPhase1Cube(const KociembaPhase1Cube& cube);

    // Destruktor.
    ~KociembaPhase1Cube();
    
    // Kopiuje stan kostki. Zwraca nowy stan, dla ktore alokuje pamiec.
    virtual Cube* Clone() { return new KociembaPhase1Cube(*this); }
    
    // Operator porownia. Porownuje ze stanem kostki cube.
    int operator==(const KociembaPhase1Cube& cube);
    
    // Ustawia stan kostki jako ulozony. Nie zwraca zadnej wartosci.
    void SetInitState();


    // Pobiera stan kostki z tabeli reprezentujacej go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    virtual void FromCubieCube(CubieCube& cube)
            throw (KociembaException);

    // Zapisuje stan kostki do tabeli
    virtual void ToCubieCube(CubieCube& cube);

    // Pobiera stan kostki ze wspolrzednych reprezentujacych go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    virtual void FromCoords(int coords[]);

    // Zapisuje stan kostki do wspolrzednych
    virtual void ToCoords(int coords[]);


    // Zamienia stan kostki na lancuch znakow reprezentujacy go. Zwraca tenze
    // lancuch znakow. Alokuje pamiec na lancuch, ktora trzeba pozniej zwolnic.
    char* ToString();

    
    // Wykonuje ruch o numerze move na kostce. Nie zwraca zadnej wartosci.
    virtual void Move(CubeMove::eMove move, CubeMove::eTurn turn);

  private:
    void MoveCoords(CubeMove::eMove move);
};


#endif // KociembaPhase1Cube_h
