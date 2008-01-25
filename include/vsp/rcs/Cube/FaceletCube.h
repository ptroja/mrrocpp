/////////////////////////////////////////////////////////
// Nazwa: FaceletCube.h
// Opis: Klasa FaceletCube opisujaca kostke na poziomie scianek.
/////////////////////////////////////////////////////////

#ifndef FaceletCube_h
#define FaceletCube_h

#include "Cube.h"
#include "CubeException.h"


// Klasa opisujaca kostke Rubika na poziomie scianek kostki. Zawiera ona
// zawartosc 54 scian kostki. Umozliwia sprawdzanie poprawnosci stanu kostki.
class FaceletCube : public Cube
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }

    
  public:
         
    // Unikalne numery scian kostki.
    enum eFace
    {
        F, R, U,   // 0 - 2
        B, L, D    // 3 - 5
    };    

    // Liczba scian kostki.
    const static int FACE_NUMBER = 6;

    // Liczba scianek w scianie.
    const static int FACELET_IN_FACE_NUMBER = 9;

    // Liczba scianek w kostce.
    const static int FACELET_NUMBER = 54;

    // 1-literowe nazwy scian kostki.
    const static char *FACE_NAMES;

    // 1-literowe kolory scian kostki.
    const static char *FACE_COLORS;

       
  private:
    
    // Zawartosc 54 scianek kostki, po 9 scianek na sciane
    eFace iaFacelets[FACE_NUMBER*FACELET_IN_FACE_NUMBER];

    // Oznakowanie 6 scian kostki na podstawie zawartosci srodkowych scianek.
    char caMarkings[FACE_NUMBER];

    
  public:
       
    // Konstruktor domyslny. Tworzy ulozona kostke.
    FaceletCube();
    
    // Konstruktor kopiujacy. Kopiuje zawartosc cube.
    FaceletCube(FaceletCube& cube);
    
    // Kopiuje stan kostki. Zwraca nowy stan, dla ktore alokuje pamiec.
    virtual Cube* Clone() { return new FaceletCube(*this); }

    // Destruktor. Pusty.
    ~FaceletCube() {}

    // Operator porownia. Porownuje ze stanem kostki cube.
    int operator==(const FaceletCube& cube);


    // Ustawia stan kostki jako ulozony. Nie zwraca zadnej wartosci.
    virtual void SetInitState();

    // Pobiera stan kostki z tabeli reprezentujacej go. Rzuca wyjatkiem 
    // w przypadku niepowodzenia.
    void FromTable(const int cube[]) 
         throw (CubeException);

    // Zapisuje stan kostki do tabeli
    void ToTable(int cube[]);

    // Pobiera stan kostki z lancucha znakow reprezentujacego go. Rzuca 
    // wyjatkiem w przypadku niepowodzenia.
    void FromString(const char* sCube) 
         throw (CubeException);

    // Zamienia stan kostki na lancuch znakow reprezentujacy go. Zwraca tenze
    // lancuch znakow. Alokuje pamiec na lancuch, ktora trzeba pozniej zwolnic.
    char* ToString();


    // Wykonuje ruch o numerze move na kostce. Nie zwraca zadnej wartosci.
    virtual void Move(CubeMove::eMove move, CubeMove::eTurn turn)
        throw (CubeException);


  private:
        
    // Sprawdza wartosci elementow tabeli, w ktorych zapisany jest stan kostki.
    void CheckTable(const int cube[]) throw (CubeException);

    // Sprawdza format lancucha znakow, w ktorym zapisany jest stan kostki.
    void CheckFormat(const char* sCube) throw (CubeException);


    // Sprawdza czy stan kostki jest poprawny. Rzuca wyjatkiem jezeli stan
    // jest niepoprawny. Nie zwraca zadnych wartosci.
    void Validate() 
         throw (CubeException);

    // Sprawdza czy oznaczenia srodkow sa prawidlowe. Oznaczenie kazdego
    // srodki powinno byc inne. Oznaczenia te sa zapamietywane jako oznaczenia
    // calych scianek.
    void ValidateCenters() throw (CubeException);

    // Sprawdza czy oznaczenia scianek sa prawidlowe. Oznaczenia powinny 
    // byc tylko takie jak oznaczenia scian. Ponadto kazde oznaczenie powinno
    // wystapic dokladnie 9 razy.
    void ValidateFacelets() throw (CubeException);


    // Znajduje sciane o nazwie name. Zwraca numer sciany. Rzuca 
    // wyjatkiem, jezeli sciana o nazwie name nie wystepuje w kostce.
    eFace FaceNameToFace(char name)
         throw (CubeException);

    // Znajduje sciane o oznakowaniu marking. Zwraca numer sciany. Rzuca 
    // wyjatkiem, jezeli oznakowanie marking nie wystepuje w kostce.
    eFace MarkingToFace(char marking)
         throw (CubeException);
         
};

#endif // FaceletCube_h
