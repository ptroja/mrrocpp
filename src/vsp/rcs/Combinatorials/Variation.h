/////////////////////////////////////////////////////////
// Nazwa: Variation.h
// Opis: Klasa Variation reprezentujaca wariacje z powtorzeniami
//       i umozliwiajaca operacje na niej.
/////////////////////////////////////////////////////////


#ifndef Variation_h
#define Variation_h

#include "CombinatorialsException.h"
#include "Collection.h"


// Klasa wariacji z powtorzeniami. Umozliwia podstawowe operacje na sobie.
class Variation : public Collection
{
      
  public:
    
    // Zwraca maksymalna obslugiwana wielkosc kolekcji. 
    static const int MAX_COLL_SIZE;

    // Zwraca maksymalna obslugiwana wielkosc zbioru, z ktorego stworzona jest
    // kolekcja. 
    static const int MAX_SET_SIZE;


  public:
  
    // Konstruktor. Tworzy wariacje zlozona z elementow o wartosci. Alokuje 
    // pamiec dla tabeli elementow wariacji. Rzuca wyjatkiem w przypadku 
    // nieprawidlowych argumentow.
    Variation(int collSize, int setSize) throw (CombinatorialsException);
    
    // Konstruktor kopiujacy. Tworzy wariacje jako kopie wariacji vari.
    Variation(const Variation& vari);


  protected:
                
    // Zwraca litere oznaczajaca wariacje. Metoda uzywana w ToString.
    virtual char GetType();

    // Sprawdza czy w tabeli tab zapisana jest poprawna wariacja. W przypadku 
    // negatywnego wyniku rzuca odpowiednim wyjatkiem.
    virtual void CheckTable(const int tab[]) throw (CombinatorialsException) {}


  public:
         
    // Tworzy kopie wariacji i zwraca do niej wskaznik.
    virtual Collection* Clone();

    
    // Ustawia wariacje zawierajaca jedynie elementy o wartosci 0.
    virtual void SetInitState();
    
    // Ustawia wariacje odpowiadajaca numerowi ord, ktory jednoznacznie 
    // identyfikuje wariacje. Metoda nie zwraca zadnych wartosci. Rzuca 
    // wyjatkiem  w przypadku nieprawidlowego argumentu.
    virtual void FromOrdinal(int ord) throw (CombinatorialsException);

    // Zwraca numer jednoznacznie identyfikujacy wariacje. 
    virtual int ToOrdinal();
    
    // Zwraca parzystosc wariacji. 
    virtual int Parity();

    // Zwraca przyzwolenie albo jego brak na skladanie wariacji z inna kolekcja.
    virtual bool AllowComposition();


    // Wylicza liczbe roznych wariacji o dlugosci collSize zbioru o setSize 
    // elementach. Rzuca wyjatkiem w przypadku nieprawidlowych argumentow.
    static int NUMBER(int collSize, int setSize) throw (CombinatorialsException);
    
    
    // Na pozycji pos wariacji wybiera element o wartosci o 1 wiekszej badz 
    // mniejszej (w zaleznosci od parametru bigger) od aktualnie wybranego. 
    // Metoda rzuca wyjatkiem w przypadku nieprawidlowego argumentu.
    void ChooseNext(int pos, bool bigger=true) throw (CombinatorialsException);
    

  private:

    // Sprawdza czy pozycja pos istnieje w wariacji dlugosci size. Pozycja 
    // w wariacji numerowane sa od 0 do size-1. W przypadku niepomyslnej 
    // weryfikacji metoda rzuca wyjatkiem.
    void ValidatePosition(int pos, int size) throw (CombinatorialsException);
    
};

#endif // Variation_h
