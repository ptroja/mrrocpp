/////////////////////////////////////////////////////////
// Nazwa: DataTable.h
// Opis: Klasa abstrakcyjna DataTable reprezentujaca tablice
//       zawierajaca dane statyczne, zapisywane do pliku.
/////////////////////////////////////////////////////////


#ifndef DataTable_h
#define DataTable_h

#include <string.h>
#include <stdio.h>


// Abstrakcyjna klasa bazowa dla klas tablic przeksztalcen i wartosci 
// heurystycznych. Dostarcza mechanizm inicjalizacja oraz zapisu i odczytu
// z/do pliku. W klasach pochodnych musi zostac zdefiniowana metoda wypelniania
// tabeli danymi.
class DataTable
{
  protected:
          
    // Wielkosc tabeli
    int iSize;

    // Wskaznik do tabeli
    int *iaTable;


  protected:
            
    // Znacznik czy wyswietlac komentarze
    static bool bLog;
    
    // Nazwa folderu, do ktorego zapisywane sa wszystkie tabele danych.
    static char* sFolderName;


  public:
    
    // Konstruktor. Alokuje odpowiednia ilosc pamieci na tabele.
    DataTable(int size);

    // Konstruktor kopiujacy.
    DataTable(DataTable& table);

    // Destruktor. Zwalnia pamiec zaalokowana na tabele.
    virtual ~DataTable();
        
    // Konstruktor kopiujacy.
    DataTable& operator=(DataTable& table);

    // Inicjalizuje tabele. Jezeli tabela (tableName) zostala 
    // wczesniej stworzona i zapisana do pliku (fileName), to tylko ja 
    // odczytuje. W przeciwnym przypadku generuje ja i zapisuje do pliku.
    void Init();
    
    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName() = 0;
    
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription() = 0;

    // Zwraca rozszerzenie pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetExtention() = 0;
    
    
    
    // Ustawia nazwe folderu, do ktorego zapisywane sa wszystkie tabele danych.
    static void SetFolderName(const char* folder) 
    {
        if (sFolderName) delete[] sFolderName;
        sFolderName = new char[strlen(folder)+1];
        strncpy(sFolderName, folder, strlen(folder)+1);
    }

    // Zwraca nazwe folderu, do ktorego zapisywane sa wszystkie tabele danych.
    static const char* GetFolderName() 
        { return ( sFolderName ? sFolderName : "" ); }

    // Zwalnia pamiec zaalokowana dla nazwy folderu.
    static void ClearFolderName() 
    {
        if (sFolderName) delete[] sFolderName;
        sFolderName = NULL;
    }

    // Ustawia znacznik logowania wiadomosci na ekran.
    static void SetLog(bool log)
        { bLog = log; }


  protected:
            
    // Udostepnia wpis z tabeli o wspolrzednej coord.
    int GetData(int coord)
        { return iaTable[coord]; }
    
    // Wpisuje dane do tabeli o wspolrzednej coord.
    void SetData(int coord, int data)
        { iaTable[coord] = data; }


  private:
    
    // Generuje tabele. Dla kazdej wartosci wspolrzednej obliczane i zapisywane
    // do tabeli jest jej przeksztalcenie przez 6 podstawowych ruchow. 
    virtual void Generate() = 0;
    
    // Odczytuje tablice ze strumienia wejsciowego pliku inputStream.
    void Load(FILE *inputStream);
    
    // Zapisuje tablice do strumienia wyjsciowego pliku outputStream.
    void Save(FILE *outputStream);
    
};


#endif // DataTable_h

