/////////////////////////////////////////////////////////
// Nazwa: Combination.h
// Opis: Klasa Combination reprezentujaca kombinacje bez 
//       powtorzen i umozliwiajaca operacje na niej.
/////////////////////////////////////////////////////////

#ifndef Combination_h
#define Combination_h

#include "CombinatorialsException.h"
#include "Collection.h"


// Klasa kombinacji bez powtorzen. Umozliwia podstawowe operacje na sobie.
class Combination : public Collection
{
      
  public:
    
    // Zwraca maksymalna obslugiwana wielkosc kolekcji. 
    static const int MAX_COLL_SIZE;

    // Zwraca maksymalna obslugiwana wielkosc zbioru, z ktorego stworzona jest
    // kolekcja. 
    static const int MAX_SET_SIZE;


  public:
  
    // Konstruktor. Tworzy permutacje identycznosciowa zbioru N-elementowego 
    // oraz typu type. Alokuje pamiec dla tabeli elementow permutacji. Rzuca 
    // wyjatkiem w przypadku nieprawidlowego argumentow.
    Combination(int collSize, int setSize) throw (CombinatorialsException);
    
    // Konstruktor kopiujacy. Tworzy permutacje jako kopie permutacji perm.
    Combination(const Combination& comb);


  protected:
                
    // Zwraca litere oznaczajaca typ kolekcji. Metoda uzywana w ToString.
    virtual char GetType();

    // Sprawdza czy w tabeli tab zapisana jest poprawna kombinacja. W przypadku 
    // negatywnego wyniku rzuca odpowiednim wyjatkiem.
    virtual void CheckTable(const int tab[]) throw (CombinatorialsException);


  public:
         
    // Tworzy kopie kolekcji i zwraca do niej wskaznik.
    virtual Collection* Clone();

    
    // Ustawia permutacje identycznosciowa.
    virtual void SetInitState();
    
    // Ustawia permutacje odpowiadajaca numerowi ord, ktory jednoznacznie 
    // identyfikuje permutacje. Metoda nie zwraca zadnych wartosci. Rzuca 
    // wyjatkiem  w przypadku nieprawidlowego typu permutacji badz argumentu.
    virtual void FromOrdinal(int ord) throw (CombinatorialsException);

    // Zwraca numer jednoznacznie identyfikujacy permutacje. 
    virtual int ToOrdinal();
    
    // Zwraca parzystosc permutacji. 
    virtual int Parity();

    // Zwraca przyzwolenie albo jego brak na skladanie kolekcji z inna kolekcja.
    virtual bool AllowComposition();


    // Wylicza liczbe roznych permutacji zbioru o size elementach. Rzuca 
    //wyjatkiem w przypadku nieprawidlowego argumentu.
    static int NUMBER(int collSize, int setSize) throw (CombinatorialsException);


  private:

    // Sprawdza poprawnosc tabeli comb reprezentujacej kombinacje zbioru 
    // o collSize elementach. Elementy comb powinny przyjmowac rozne wartosci
    // ze zbioru 0 do setSize-1. W przypadku niepomyslnej weryfikacji metoda 
    // rzuca wyjatkiem.
    static void ValidateCombination(const int comb[], int collSize, int setSize) throw (CombinatorialsException);
    
};

#endif // Combination_h
