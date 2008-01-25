#define MAKS_LA_KLAS 200
#define MAKS_LA_PROBEK 10
#define MAKS_LA_PFONEMOW 1024

// STA£A dla normalizacji œredniej energii w oknie sygna³u
#define NORMALNA_ENERGIA 6000.0
#define WSPOLCZ_PREEMFAZY 0.9
#define MAKS_BLAD_KODERA 6.0

class CROSMDoc
{
public:
	//fields
	int		LA_KLAS; // Liczba klas (komend)
	int		LA_PROBEK; // Liczba próbek (plików) dla pojedynczej komendy
	long	LA_KOLUMN; // Liczba wetorów cech dla kwantyzacji 
	int		LA_PFONEMOW; // Liczba klas fonetycznych

	// int		LICZ_SPEKTRO; // Prze³¹cznik: obliczenie czy tylko wizualizacja?
	int		NORMALIZUJ_SPEKTROGRAM; // Normalizacja wzglêdem najwiêkszego elementu

	//ParametersDialog paramDialog; //WINDOWS

	double * u[MAKS_LA_KLAS]; // Beda tu dane modelu dla maksymalnej liczby komend
	double * s[MAKS_LA_KLAS]; // --"--
	double * p[MAKS_LA_KLAS]; // --"--

	double Smin; // Parametry stopu procesu uczenia
	double stop; // --"--
	// int M;
	
	// Tablice nazw dla automatyzacji dostêpu do danych 
	//CString NazwaSciezki, NazwaKatalogu; //WINDOWS
	//CString NazwaKlasy[MAKS_LA_KLAS]; //WINDOWS
	//CString NazwaKomendy[MAKS_LA_KLAS]; //WINDOWS

	// Tablice danych dla uczenia modeli komend
	double* licznikiSpekt[MAKS_LA_KLAS]; // Sumaryczne dane dla komend
	int mianownikiSpekt[MAKS_LA_KLAS]; // Sumaryczne dane dla komend
	double *spektrogramySrednie[MAKS_LA_KLAS]; // Srednie widma dla komend

	// Tablice danych dla kodowania podfonemów
	double* mozliweProbki; // Wszystkie wektory cech wszystkich próbek wszystkich komend
	double** probkiPFonemow;// Wska¿niki do kolumn - indywidualnych próbek
	int* klasaProbki; // Indeks klasy podfonemu dla próbki
	double* stdDevCech; // £¹czny wektor odchyleñ standardowych dla cech próbek 
	double* reprezenCechPFonemow; // Reprezentacyjne wektory cech dla klas podfonemow
	double** reprezenPFonemow; // WskaŸniki do cech dla klas podfonemow
	
		
	// Dla aktualnego pliku wav
	double *spektA; // Tu bêd¹ 2 ramki cech MFCC
	double *spektAszer; // Tu bêd¹ 2 ramki szerokiego widma amplitudowego
	double *windowFun; // Tu bêdzie funkcja okna Hamminga
	double *filtryMEL; // Tu bêd¹ centra filtrów pasmowych wed³ug MEL skali
	int *poczMEL, *koniecMEL;
	double** wspolczMEL; 
	double* wspolczMELWszystkie;
	int  liczbaCechMFC, lCechMFCC;

	double czestotliwoscProbkowania, czestotliwoscBazowa; // Czestotliwosci zalezne od pliku i parametrow FFT

	int wierszeUproszczonego; // Rozmiar y ramki cech MFCC
	int kolumnyUproszczonego; // Rozmiar x ramki cech MFCC
	int kolumnyUproszcz; // Rozmiar x widma amplitudowego
	int okno;  // Liczba probek w 1-ym oknie = szerokosc transformaty FFT
	int odstepOkna; // Odstep pomiedzy 2 kolejnymi oknami
	int waska_ramka; // Po³o¿enie x znalezionej ramki w podwojnej ramce szerokiej
	long poczatek_ramki; // pocz¹tek u¿ytecznego sygnalu (szerokiej ramki)
	short typSpektrogramu; // Typ wyswietlanego spektrogramu: 0-brak, 1- cechyMFCC ,2- szerokie widmo 
	//WAVEFORMATEX waveFormat; //WINDOWS

	int liczba_probekBezCiszy;
	bool otwarty;
	bool bezSzumu; // Parametr przetwarzania: przed analiza wstepna (false) lub po niej (true)

	// Dla wspó³pracy z plikami wav
	//CWave tempWave; //WINDOWS
	//CWaveIn waveIn; //WINDOWS
	//CWaveDevice myDevice; //WINDOWS
	//CWaveOut waveOut; //WINDOWS
	//CString m_fileName; // Nazwa pliku wav //WINDOWS
	
	// Dla pliku wav - dane wejsciowe do analizy
	int liczba_probek; // Dla aktualnej probki
	double *probki; // Tu beda probki sygnalu w czasie
	double *probkiBezCiszy; // Fragment pozostaly po usunieciu ciszy - poczatek wycinka sygnalu
	double *probkiEnergia; // Dla obliczenia energii poszczegolnych probek
	double poziomCiszy;	// prog dla amplitudy sygna³u
	double maxWartosc;
	//CWaveBuffer bufor; //WINDOWS

	char* recog_word; //rozpoznane slowo
	int recog_word_id; //index 

	//methods
	CROSMDoc(); //ROSMDoc.cpp
	~CROSMDoc();
	void InitObiekt(int czProbek, int pCiszy, double wspOdstepuOkna, int lOkien, int wspRedukcjiWierszy, int lCechMFC,
							  char* nKatalogu, char* nSciezki);
	void UstawTablice();
	void UsunTablice();
	void InicjujAnalize(int bezPrzerw);
	long ZnajdzRamke(int szeroki);
	void ObliczCechyOkien(int typ_spektro, int bezPrzerw);
	void WczytajKlasy(); //WeWyKlas.cpp
	void RozpoznajKomende(); //KlasyfikacjaSpektr.cpp
	int WykonajKlasyfikacje(double* ymin);
};
