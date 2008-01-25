/////////////////////////////////////////////////////////
// Nazwa: Exception.h
// Opis: Klasa bazowa Exception dla wszystkich bledow.
/////////////////////////////////////////////////////////

#ifndef Exception_h
#define Exception_h

#include <stdio.h>
#include <string.h>


// Klasa bazowa dla wszystkich bledow.
class Exception
{

  public:

    // Zwraca typ klasy.
    virtual const char* GetType() = 0;

  protected:

    // Kod bledu.
    int iError;

    // Konstruktor. Tworzy obiekt dla okreslonego bledu.
    Exception(int error) : iError(error) {};

  public:
         
	virtual ~Exception() {};

    // Kopiuje blad. Zwraca nowy blad, dla ktorego alokuje pamiec.
    virtual Exception* Clone() = 0;

    // Zrzuca zawartosc bledu do lancucha znakow. Zajmuje pamiec dla zwracanego
    // lancucha, ktora nalezy pozniej zwolnic.
    virtual char* ToString()
    {
        const int STR_LEN = 100;
        char *sExp = new char[STR_LEN];
        int pos = 0;

        // set exception type
        strncpy(sExp, "TYPE=", 5);
        pos += 5;
        const char *type = GetType();
        strncpy(&sExp[pos], type, strlen(type));
        pos += strlen(type);
        
        // set error code and text
        strncpy(&sExp[pos], " ERR=[000] TXT=", 15);
        sExp[pos+8] = '0' + iError%10;
        sExp[pos+7] = '0' + iError/10%10;
        sExp[pos+6] = '0' + iError/100;
        pos += 15;
        const char *txt = GetErrorText();
        strncpy(&sExp[pos], txt, ( ((int) strlen(txt))>STR_LEN-pos ? STR_LEN-pos : strlen(txt)+1 ));
        sExp[STR_LEN-1] = '\0';
        
        return sExp;
    }
    
    // Zwraca tekst bledu.
    virtual const char* GetErrorText() = 0;
    
    const int GetError() { return iError; }

};

#endif // Exception_h        
