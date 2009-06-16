/////////////////////////////////////////////////////////
// Nazwa: Tester.h
// Opis: Klasa Tester bazowa dla wszystkich klas testow.
/////////////////////////////////////////////////////////

#ifndef Tester_h
#define Tester_h


// Klasa testow.
class Tester
{

  public:
  
    virtual bool Test() = 0;

	virtual ~Tester() {};

    static void SetLog(bool log) { bLog = log; } 
    static void SetRepetitions(int rep) { iRep = rep; } 

  protected:
    static bool bLog;
    static int iRep;

    const char* ResultText(bool result) { return ( result ? "OK" : "FAILED" ); }
};


#endif // CombinatorialsTester_h
