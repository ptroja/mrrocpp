/////////////////////////////////////////////////////////
// Nazwa: Permutation.h
// Opis: Klasa Permutation reprezentujaca permutacje bez 
//       powtorzen i umozliwiajaca operacje na niej.
/////////////////////////////////////////////////////////


#ifndef Permutation_h
#define Permutation_h

#include "CombinatorialsException.h"
#include "Collection.h"


// Klasa permutacji bez powtorzen. Umozliwia podstawowe operacje na sobie.
class Permutation : public Collection
{
      
  public:
    
    // Zwraca maksymalna obslugiwana wielkosc kolekcji i jednoczesnie
    // wielkosc zbioru, z ktorego stworzona jest kolekcja. 
    static const int MAX_SIZE;


  public:
  
    // Konstruktor. Tworzy permutacje identycznosciowa zbioru o size elementach. 
    // Alokuje pamiec dla tabeli elementow permutacji. Rzuca wyjatkiem w 
    // przypadku nieprawidlowego argumentu.
    Permutation(int size) throw (CombinatorialsException);
    
    // Konstruktor kopiujacy. Tworzy permutacje jako kopie permutacji perm.
    Permutation(const Permutation& perm);


  protected:
                
    // Zwraca litere oznaczajaca permutacje. Metoda uzywana w ToString.
    virtual char GetType();


  public:
         
    // Tworzy kopie permutacji i zwraca do niej wskaznik.
    virtual Collection* Clone();

    
    // Ustawia permutacje identycznosciowa.
    virtual void SetInitState();
    
    // Ustawia permutacje odpowiadajaca numerowi ord, ktory jednoznacznie 
    // identyfikuje permutacje. Metoda nie zwraca zadnych wartosci. Rzuca 
    // wyjatkiem  w przypadku nieprawidlowego argumentu.
    virtual void FromOrdinal(int ord) throw (CombinatorialsException);

    // Zwraca numer jednoznacznie identyfikujacy permutacje. 
    virtual int ToOrdinal();
    
    // Zwraca parzystosc permutacji. 
    virtual int Parity();

    // Zwraca przyzwolenie albo jego brak na skladanie permutacji z inna kolekcja.
    virtual bool AllowComposition();


    // Wylicza liczbe roznych permutacji zbioru o size elementach. Rzuca 
    // wyjatkiem w przypadku nieprawidlowego argumentu.
    static int NUMBER(int size) throw (CombinatorialsException);


    // Ustawia permutacje odpowiadajaca C-cyklowi cycle. Rzuca wyjatkiem  
    // w przypadku niepoprawnego cyklu.
    void FromCycle(const int cycle[], int C) throw (CombinatorialsException);

    // Zamienienia permutacje na swoja odwrotnosc.
    void Inverse();


  protected:

    // Sprawdza czy w tabeli tab zapisana jest poprawna permutacja. W przypadku 
    // negatywnego wyniku rzuca odpowiednim wyjatkiem.
    virtual void CheckTable(const int tab[]) throw (CombinatorialsException);


  private:

    // Sprawdza poprawnosc C-cyklu w permutacji zbioru o size elementach. Cykl 
    // powinien byc dlugosci 1 do size, a jego elementy powinny przyjmowac rozne 
    // wartosci ze zbioru 0 do size-1. W przypadku niepomyslnej weryfikacji 
    // metoda rzuca wyjatkiem.
    static void ValidateCycle(const int cycle[], int C, int N)  throw (CombinatorialsException);

    // Sprawdza poprawnosc tabeli perm reprezentujacej permutacje 
    // zbioru o size elementach. Elementy perm powinny przyjmowac rozne wartosci
    // ze zbioru 0 do size-1. W przypadku niepomyslnej weryfikacji metoda rzuca
    // wyjatkiem.
    static void ValidatePermutation(const int perm[], int size) throw (CombinatorialsException);

};

#endif // Permutation_h
