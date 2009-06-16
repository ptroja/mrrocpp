/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase2Cube.h
// Opis: Klasa KociembaPhase2Cube zawierajaca opis kostki Rubika
//      na potrzeby algorytmu Kociemby.
/////////////////////////////////////////////////////////

#ifndef KociembaPhase2Cube_h
#define KociembaPhase2Cube_h

#include "KociembaCube.h"
#include "KociembaException.h"


class KociembaPhase2MoveTable;

// Klasa opisujaca Kostke Rubika na poziomie wspolrzednych.
class KociembaPhase2Cube : public KociembaCube 
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }

    
  private:
    int cornerPermutationCoord;
    int edgeMidSlicePermutationCoord;
    int edgeNonMidSlicePermutationCoord;
    
  private:
    static bool bInitialized;

    // Tabela przeksztalcen wspolrzednych permutacji rogow przez obroty
    // uzywana w drugiej fazie algorytmu
    static KociembaPhase2MoveTable *mtCP;

    // Tabela przeksztalcen wspolrzednych permutacji krawedzi srodkowej
    // warstwy przez obroty uzywana w drugiej fazie algorytmu
    static KociembaPhase2MoveTable *mtEMP;

    // Tabela przeksztalcen wspolrzednych permutacji krawedzi gornej i dolnej
    // warstwy przez obroty uzywana w drugiej fazie algorytmu
    static KociembaPhase2MoveTable *mtENP;

    
  public:
    static void SInit();
    static void SClear();


  public:
         
    // Konstruktor domyslny. Tworzy ulozona kostke.
    KociembaPhase2Cube();
    
    // Konstruktor kopiujacy. Tworzy nowy stan rowny stanowi cube.
    KociembaPhase2Cube(const KociembaPhase2Cube& cube);

    // Destruktor.
    ~KociembaPhase2Cube();
    
    // Kopiuje stan kostki. Zwraca nowy stan, dla ktore alokuje pamiec.
    virtual Cube* Clone() { return new KociembaPhase2Cube(*this); }
    
    // Operator porownia. Porownuje ze stanem kostki cube.
    int operator==(const KociembaPhase2Cube& cube);
    
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

    // Sprawdza, czy mozna na kostce wykonac ruch move o obrocie turn.
    virtual bool IsAllowed(CubeMove::eMove move, CubeMove::eTurn turn);


  private:
    void MoveCoords(CubeMove::eMove move);
};


#endif // KociembaPhase2Cube_h

