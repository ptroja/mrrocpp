/////////////////////////////////////////////////////////
// Nazwa: KorfException.cpp
// Opis: Klasa KorfException bledu oznaczen kostki.
/////////////////////////////////////////////////////////

#ifndef KorfException_h
#define KorfException_h

#include "../General/Exception.h"


// Klasa bledu opisu kostki Rubika.
class KorfException : public Exception
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }


  public:

    // Konstruktor. Tworzy obiekt dla okreslonego bledu.
    KorfException(int error);

    // Konstruktor kopiujacy.
    KorfException(const KorfException& exp);

    // Destruktor. Zwalnia zajmowana pamiec.
    virtual ~KorfException();
    
    // Kopiuje blad. Zwraca nowy blad, dla ktorego alokuje pamiec.
    virtual Exception* Clone();

    // Zwracane kody bledow.
    enum eError
    {
        SUCCESS,                         //  0
        ERR_SOLVER_INPUT_INVALID,        //  1
        ERR_NUMBER                       //  2  - liczba zwracanych kodow
    };

    
  protected:

    virtual const char* GetErrorText();

            
  private:

    // Wiadomosci bledow.
    static char *ErrorText[];

};


#endif // KorfException_h        
