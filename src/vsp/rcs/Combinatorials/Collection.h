/////////////////////////////////////////////////////////
// Nazwa: Collection.h
// Opis: Klasa abstrakcyjna Collection reprezentujaca kolekcje 
//       elementow i umozliwiajaca operacje na niej. Klasa 
//       bazowa dla wszystkich klas kombinatorycznych.
/////////////////////////////////////////////////////////

#ifndef Collection_h
#define Collection_h

#include "CombinatorialsException.h"


// Abstrakcyjna klasa kolekcji elementow. Umozliwia podstawowe operacje na sobie.
// Klasa bazowa dla wszystkich klas kombinatorycznych.
class Collection
{
      
  protected:
          
    // Znacznik trybu pracy - z walidacja stanu czy bez.
    static bool bValidation;
    
    
  public:
         
    // Ustawia tryb pracy.
    static void SetValidation(bool on) { bValidation = on; }


  protected:
          
    // Tabele zawierajaca elementy kolekcji.
    int *iaElems;
    
    // Wielkosc kolekcji.
    int iCollSize;

    // Wielkosc zbioru, na ktorym utworzona jest kolekcja.
    int iSetSize;

    
  protected:
  
    // Konstruktor. Tworzy kolekcje o size elementach. Alokuje pamiec dla tabeli 
    // swoich elementow. Rzuca wyjatkiem w przypadku nieprawidlowego argumentow.
    Collection(int collSize, int setSize, int maxCollSize, int maxSetSize) 
        throw (CombinatorialsException);
    
    // Konstruktor kopiujacy. Tworzy kolekcje jako kopie kolekcji coll.
    Collection(const Collection& coll);

    // Konstruktor przypisania.
    Collection& operator=(const Collection& coll);


  public:
           
    // Destruktor. Zwalnia pamiec tabeli elementow kolekcji.
    virtual ~Collection();
    
    // Operator porowniania dwoch kolekcji. Zwraca 0, jezeli kolekcje
    // sa takie same.
    int operator==(const Collection& coll);
    
    // Tworzy kopie kolekcji i zwraca do niej wskaznik.
    virtual Collection* Clone() = 0;

    
    // Ustawia stan poczatkowy kolekcji.
    virtual void SetInitState();
    
    // Ustawia stan kolekcji odpowiadajacy numerowi ord, ktory jednoznacznie 
    // identyfikuje kolekcje. Metoda nie zwraca zadnych wartosci. Metoda rzuca 
    // wyjatkiem  w przypadku nieprawidlowego argumentu.
    virtual void FromOrdinal(int ord) throw (CombinatorialsException) = 0;

    // Zwraca numer jednoznacznie identyfikujacy kolekcje. 
    virtual int ToOrdinal() = 0;

    // Ustawia stan kolekcji odpowiadajacy ulozeniu elementow w tabeli tab.
    // Tabela musi byc odpowiedniej dlugosci (iCollSize). Metoda nie zwraca 
    // zadnych wartosci. Metoda rzuca wyjatkiem, jezeli elementy w tabeli nie 
    // reprezentuja poprawnej kolekcji.
    virtual void FromTable(const int tab[]) throw (CombinatorialsException);

    // Wypelnia tabele tab elementami kolekcji. Tabela musi byc odpowiedniej 
    // wielkosci (iCollSize).
    virtual void ToTable(int tab[]);

    // Zwraca parzystosc kolekcji. 
    virtual int Parity() = 0;

    // Sklada kolekcje z kolekcja coll. Kolekcje musza byc rownoliczne. 
    // Ponadto coll musi pozwalac na skladanie jej. W przypadku niezgodnosci 
    // metoda rzuca wyjatkiem. 
    void Composite(Collection& coll) throw (CombinatorialsException);

    // Zwraca przyzwolenie albo jego brak na skladanie kolekcji z inna kolekcja.
    virtual bool AllowComposition() = 0;
    
    // Zamienia kolekcje na lancuch znakow reprezentujacy ja. Alokuje pamiec 
    // na lancuch, ktora trzeba pozniej zwolnic. Zwraca tenze lancuch znakow. 
    char* ToString();


  protected:
    
    // Zwraca litere oznaczajaca typ kolekcji. Metoda uzywana w ToString.
    virtual char GetType() = 0;

    // Sprawdza w tabeli tab zapisana jest poprawna kolekcja. W przypadku 
    // negatywnego wyniku rzuca odpowiednim wyjatkiem.
    virtual void CheckTable(const int tab[]) throw (CombinatorialsException) = 0;

    // Sprawdza poprawnosc wielkosci kolekcji i wielkosci zbioru, z ktorego 
    // utworzona jest kolekcja. Wielkosci te powinny byc z zakresu od 1 do max. 
    // Do niektorych operacji dozwolona jest rowniez wartosc size rowna 0.
    static void ValidateSize(int size, int max, bool zero=false) 
           throw (CombinatorialsException);

    // Sprawdza poprawnosc numer ord, ktory jednoznacznie reprezentuje 
    // kolekcje zbioru o size elementach. Numer ord powinien przyjmowac wartosci
    // z zakresu 0 do max-1. Dla innych wartosci metoda rzuca wyjatkiem.
    static void ValidateOrdinal(int ord, int max) 
           throw (CombinatorialsException);
    
    // Sprawdza poprawnosc kolekcji zapisanej w tabeli tab. Kolekcja powinna 
    // byc wielkosci colSize i powinna zawierac elementy zbioru o setSize 
    // elementach. W przypadku niepomyslnej weryfikacji rzuca wyjatkiem.
    static void ValidateCollection(const int tab[], int collSize, int setSize) 
           throw (CombinatorialsException);

};

#endif // Collection_h
