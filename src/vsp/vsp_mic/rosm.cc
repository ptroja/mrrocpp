#include "vsp/rosm.h"


//M1W4-6L1Y-H82Y-T6BK-UNPW

#include "fft.cc"

double dopasowanie1(double *vd, double *vref, int len, int frameIndex, int IFVISUAL)
{
	double mvd, mvref, iloczyn, value, value1, value2;
	const double MIN_VEKTOR = 0.0001;
	// const double MIN_COMPONENT = .01;
	const double MAX_COMPONENT = 10000.0;
	const double BAD_CORREL_PENALTY = 2.0;

	/* Parametr len to po�owa element�w wektora cech */
	int length = len + len;
	int i;
	int K = len; // Indeks dla gradientu zerowego wsp�czynnika
	double MIN_COMPONENT;
	double deltaK = 0.2 * K;

	double *diff = new double[length];

	///
	if (len>1)
		MIN_COMPONENT = 1.0 / length;
	else
		MIN_COMPONENT = 0.1;

	/* Oblicz  sum� r�nic wzgl�dnych wsp�czynnik�w 2 wektor�w */

	iloczyn = 0.0;
	value1 = fabs(vref[0]);
	// value2 = fabs(vd[0]);
	value = value1; // Liczymy wzgl�dem referencyjnej warto�ci - energii referencyjnego wektora
	if (value < MIN_COMPONENT)
		value = MIN_COMPONENT;

	diff[0] = length * fabs(vd[0] - vref[0]) / value; // 1) problematyczne dla kr�tkich s��w
	//diff[0] = fabs(vd[0] - vref[0]); // 2) bezwzgl�dne por�wnanie energii
	iloczyn = diff[0];

	for (i=1; i < K; i++)
	{
		value1 = fabs(vref[i]);
		// value2 = fabs(vd[i]);

		value = value1; // 1)
		if (value < MIN_COMPONENT)
			value = MIN_COMPONENT;

		diff[i] = fabs(vd[i] - vref[i])/ value; // 1) value jest wsp�czynnikiem
		// diff[i] = fabs(vd[i] - vref[i]) * value ; // 2) bezwzgl�dne por�wnanie - value jest tu zerowym wsp.

		if ((vd[i] * vref[i]) < 0) // ewentualnie dodaj kar� za niezgodno�� znak�w
			diff[i] = diff[i] * BAD_CORREL_PENALTY; // kara za ujemn� korelacj�

		iloczyn += diff[i];
	}

	/////////////////
	/* Dla K-tego wsp�czynnika - gradientu energii okna */
	value1 = fabs(vref[K]);
	value2 = fabs(vd[K]);
	value = value1 + value2; // 1)

	if (value < MIN_COMPONENT)
		value = MIN_COMPONENT;

	diff[K] =  fabs(vd[K] - vref[K])/ value; // 1)
	// diff[K] =  fabs(vd[K] - vref[K]) * value; // 2) bezwzgl�dne warto�ci

	// diff[K] = 0.0; // wymuszenie zera
	iloczyn += diff[K];

	/////////////////////
	/* Dla gornej czesci - gradientow */
	for (i=K+1; i < length; i++)
	{
		value = fabs(vref[i]) + fabs(vd[i]); // 1)

		if (value < MIN_COMPONENT)
			value = MIN_COMPONENT;

		diff[i] = 0.1 * fabs(vd[i] - vref[i]) / value; // 1)
		// diff[i] = fabs(vd[i] - vref[i]) * value; // 2)
		// diff[i] = 0.0;
		iloczyn += diff[i];
	}

	if (iloczyn < 0.00000000001)
		iloczyn = 10000000000.0;
	else
		iloczyn = 1.0 /iloczyn;

#ifdef MONITOR
	if (IFVISUAL == 1)
	{
		CString tmpstr;
		CString info;
		info.Format("Ramka %d,  r�nica %.1f lub jako�� %f\n", frameIndex, 1/iloczyn, iloczyn);
		for (i=0; i< length; i++)
		{
          tmpstr.Format("%d\t-> %f | %f = %f\n", i, vd[i], vref[i], diff[i] );
		  info += tmpstr;
		}
		AfxMessageBox(info);
	}
#endif

	delete [] diff;

	return iloczyn;
}

double penalty1( double *spek, int najkol, int wiersze, int kolumny )
{
	double kara;
	int i, ostatnia;
	long j;

	j=0;
	kara = 0.0;
	for (i=0; i<najkol; i++) // Przed ramka
	{
		kara += fabs(spek[j]);
		j += wiersze;
	}
	ostatnia = kolumny + kolumny;
	j = wiersze * (najkol + kolumny);
	for (i=najkol + kolumny; i < ostatnia; i++) // Po ramce
	{
		kara += fabs(spek[j]);
		j += wiersze;
	}



	return kara;
}

double suma2Ramek( double *spek, int wiersze, int dwieKolumny )
{
	double suma;
	int i;
	long j;

	j=0;
	suma = 0.0;
	for ( i=0; i< dwieKolumny; i++) // Podwojna ramka
	{
		suma += fabs(spek[j]);
		j += wiersze;
	}

	return suma;
}

CROSMDoc::CROSMDoc()
{
	// Use OLE compound files
	// EnableCompoundFile(); //WINDOWS

	std::cout << "Constructor CROSMDoC" << std::endl;

	// TODO: add one-time construction code here
    //InitObiekt(22000, 6000, 0.5, 28, 1, 32, "001m",
	//	"C:/automatyka2/MStaniak/RozpKom2/DATA");  //dla f=22kHz

	InitObiekt(16000, 6000, 0.5, 32, 1, 24, (char*)"P16",
			(char*)"C:/automatyka2/MStaniak/RozpKom2/DATA");  //dla f=16kHz

	//InitObiekt(8000, 6000, 1.0, 24, 1, 12, "Probki",
	//	"C:/automatyka2/MStaniak/RozpKom2/DATA");  //dla f=8kHz


	// Inicjalizacja pami�ci i parametr�w dla obiektu - dokumentu
}
CROSMDoc::~CROSMDoc() // Destruktor - zwolnij pami�c dynamiczn� zwi�zan� z obiektem dokumentu
{
	delete [] probki;
	delete [] probkiEnergia;
	delete [] probkiBezCiszy;

	UsunTablice(); // Dla uczenia cech

	delete [] mozliweProbki; // Wszystkie wektory cech wszystkich pr�bek wszystkich komend
	delete [] probkiPFonemow;// Wska�niki do kolumn - indywidualnych pr�bek
	delete [] klasaProbki; // Indeks klasy podfonemu dla pr�bki
	delete [] stdDevCech; // ��czny wektor odchyle� standardowych dla cech pr�bek
	delete [] reprezenCechPFonemow; // Reprezentacyjne wektory cech dla klas podfonemow
	delete [] reprezenPFonemow; // Wska�niki do cech dla klas podfonemow

	// Dla filtr�w w MEWL skali
	delete [] windowFun;
	delete [] filtryMEL;
	delete [] poczMEL;
	delete [] koniecMEL;
	delete [] wspolczMEL;
	delete [] wspolczMELWszystkie;

	delete [] spektA;
	delete [] spektAszer;

	for (int i=0; i< MAKS_LA_KLAS; i++)
		{
			delete [] spektrogramySrednie[i];
			delete [] licznikiSpekt[i];
		}

}

void CROSMDoc::InitObiekt(int czProbek, int pCiszy, double wspOdstepuOkna, int lOkien, int wspRedukcjiWierszy, int lCechMFC,
						  char* nKatalogu, char* nSciezki)
{

	// Domy�lne ustawienie parametr�w
	poziomCiszy = pCiszy; // prog dla amplitudy "niezerowego" sygna�u
	typSpektrogramu = 0;  // na pocz�tku nie ma jeszcze spektrogramu ani tablicy cech

	if ((czProbek >= 12000) && ( czProbek <= 18000))
	{
		okno = 256; // Domy�lny rozmiar dla 16 kHz.
		czestotliwoscProbkowania = czProbek;
	}
	if (czProbek < 12000)
	{
		okno = 128; // Domy�lny rozmiar dla 8 kHz.
		czestotliwoscProbkowania = czProbek;
	}
	if (czProbek > 18000)
	{
		okno = 512; // Domy�lny rozmiar dla 22 kHz.
		czestotliwoscProbkowania = czProbek;
	}

	odstepOkna = (int)( okno * wspOdstepuOkna); // okna zachodz� na siebie lub nie


	// Liczka okien w ramce sygna�u zawieraj�cej komend�
	kolumnyUproszczonego= lOkien; // 40 to najlepsza liczba okien przy odstepie 64 dla 128

	// wierszeUproszczonego to liczba wspolczynnikow widma po redukcji
	if (wspRedukcjiWierszy >= 1) // maksymalnie 1/2 wsp�czynnik�w
		wierszeUproszczonego = (int) (okno * 0.5/wspRedukcjiWierszy);
	else
		wierszeUproszczonego = (int) (okno * 0.5); // maksymalna liczby wsp�czynnik�w

	// Liczba filtr�w tr�j�tnych wed�ug MEL-skali
	if (lCechMFC <= wierszeUproszczonego)
		liczbaCechMFC = lCechMFC;
	else liczbaCechMFC = wierszeUproszczonego;

	// Liczba cech MFCC - standardowo: energia + 12 cech = 13 pozycji
	if (wierszeUproszczonego > 24)
		lCechMFCC = 13;
	else lCechMFCC = (wierszeUproszczonego / 2);

	//NazwaKatalogu = nKatalogu; // Katalog z danymi - moje nagrania dla 8000 Hz //WINDOWS
	//NazwaSciezki = nSciezki; // �cie�ka dost�pu do pr�bek sygna�u mowy //WINDOWS

	kolumnyUproszcz = kolumnyUproszczonego * 2; // Tu ustalamy ewentualna redukcje liczby kolumn

	LA_KLAS = 1; // Liczba klas wynosi na wstepie 1 (1 to dlatego bo pomijamy dane dla indeksu 0)
	LA_PROBEK = 1; // Liczba plik�w dla jednej komendy

	LA_KOLUMN = 0; // Liczba wektor�w cech dla kwantyzacji
	LA_PFONEMOW = 0; // Liczba klas podfonemow

	//LICZ_SPEKTRO = 1; // Policz spektrogram
	//NORMALIZUJ_SPEKTROGRAM = 1; // Tak - normalizacja spektrogramu wzgl�dem najwiekszej warto�ci.
	NORMALIZUJ_SPEKTROGRAM = 0; // Nie - nie normalizuj spektrogramu


	// Oblicz i stablicuj funkcje okna Hamminga
    int okno1 = okno-1;
	const double pipi = 2 * 3.141592653589793; // sta�a 2 pi
	windowFun = new double[okno];
	int k=0;
	for (k=0; k< okno; k++)
		windowFun[k] = 0.54 - 0.46 * cos (pipi * k / okno1);

	// Alokacja na sta�e pami�ci dynamicznej dla cech ramek sygna�u ...
    spektA = new double[wierszeUproszczonego * kolumnyUproszczonego * 2]; // dla 2-ramkowej tablicy cech
	// ... poniewaz sa klopoty ze zwalnianiem pami�ci dynamicznej pomi�dzy watkami.
	spektAszer = new double[okno * kolumnyUproszcz * 2]; // Dla pelnego 2-ramkowego spektrogramu
	// Lepsze rozwi�zanie - utworzenie dokumentu dla kazdego otwieranego pliku (WK)
	// jendak na razie nie zrealizowane (WK).

	liczba_probek = okno * 2 * kolumnyUproszcz; //
	liczba_probekBezCiszy = liczba_probek; // Na wszelki wypadek dla szerokiej ramki
	probkiBezCiszy = new double[liczba_probekBezCiszy]; // Ta alokacja pozostanie niezmienna

	probki = new double[liczba_probek]; // Ta alokacja jest tylko tymczasowa - zalezy od pliku wav
	probkiEnergia = new double[liczba_probek]; // Ta alokacja tez zalezy od aktualnej wielkosci pliku wav

	int i=0;
	for (i=0; i<liczba_probek; i++)
		probki[i]=0;  // WK
	maxWartosc=1.0;
	bezSzumu = false;
	otwarty=false;

	waska_ramka = 0; // Na razie waska ramka polozona jest domyslnie na poczatku szerokiej ramki

	///////////////
	// Alokacja pami�ci dla modelowych cech wszystkich mo�liwych komend
	long rozmiar_spektra = kolumnyUproszczonego * wierszeUproszczonego;
	for (i=0; i< MAKS_LA_KLAS; i++)
	{
		mianownikiSpekt[i]=0;
		spektrogramySrednie[i]= new double[rozmiar_spektra];
		licznikiSpekt[i]= new double[rozmiar_spektra];
		for(long j=0; j< rozmiar_spektra; j++)
			spektrogramySrednie[i][j] = licznikiSpekt[i][j] = 0.0;
	}
	//
	// Alokacja pami�ci dla wszystkich cech - dla kodera podfonem�w
	//
	mozliweProbki = new double[MAKS_LA_KLAS * MAKS_LA_PROBEK * rozmiar_spektra];
	long liczbaMozliwychProbek = MAKS_LA_KLAS * MAKS_LA_PROBEK * kolumnyUproszczonego;
	probkiPFonemow = new double*[liczbaMozliwychProbek];
	double* ptr= mozliweProbki;
	for (long j=0; j < liczbaMozliwychProbek; j++, ptr+=wierszeUproszczonego)
		probkiPFonemow[j] = ptr;

	klasaProbki = new int[liczbaMozliwychProbek];
	stdDevCech = new double[wierszeUproszczonego];

	reprezenCechPFonemow = new double[MAKS_LA_PFONEMOW * wierszeUproszczonego];
	reprezenPFonemow = new double*[MAKS_LA_PFONEMOW];
	ptr= reprezenCechPFonemow;
	for (long j=0; j < MAKS_LA_PFONEMOW; j++, ptr+=wierszeUproszczonego)
		reprezenPFonemow[j] = ptr;


    ///////////////////////////////////////////////////////
	//
	// Filtry w MEL-skali
	//
	int polowaWierszy = wierszeUproszczonego;
	filtryMEL = new double[liczbaCechMFC + 2];
	poczMEL= new int[liczbaCechMFC + 1];
	koniecMEL= new int[liczbaCechMFC + 1];

	int rozmiarFiltrow = (liczbaCechMFC+1) * (polowaWierszy+1);
	wspolczMELWszystkie = new double[rozmiarFiltrow];
	wspolczMEL = new double*[liczbaCechMFC+1];
	ptr = wspolczMELWszystkie;
	for (i=0; i<=liczbaCechMFC; i++, ptr+=(polowaWierszy+1))
		wspolczMEL[i] = ptr;


	// W skali MEL czemu odpowiada maksymalna czestotliwosc (1/2 f_probkowania)?
	czestotliwoscBazowa = 2595 * log10 (1.0 + (czestotliwoscProbkowania * 0.5) / 700);
	// Ile wynosi czestotliwosc bazowa w skali MEL?
	czestotliwoscBazowa = czestotliwoscBazowa / liczbaCechMFC ;

	// Oblicz �rodki filtrow w oryginalnej skali wtedy, gdy sa one rozmieszczone rownomiernie w MEL skali
	filtryMEL[0] = 0.0;
	for (k=1; k <= liczbaCechMFC + 1; k++)
		filtryMEL[k] = 700.0 * (pow(10.0, (czestotliwoscBazowa * k) / 2595.0) - 1.0);
	// Przekszta�camy kolejne wielokrotnosci czestotliwosci podstawowej na MEL skale

	// Teraz czestotliwoscBazowa bedzie odnosic sie do wspolczynnikow szeregu Fouriera
	czestotliwoscBazowa = czestotliwoscProbkowania / okno;

	// Liczymy przynaleznosc wspolczynnikow Fouriera do kolejnych filtrow pasmowych
	poczMEL[0] = 0;
	double ftmp, fact = czestotliwoscBazowa;
	//poprzedni = 1;
	koniecMEL[0] = 1;

	ptr = wspolczMEL[0];
	for (k=0; k < rozmiarFiltrow; k++, ptr++) // Na pocz�tek wyzeruj wszystkie wzmocnienia
		*ptr = 0.0;

	int M;
	int L = 1; // Zacznij uwzgledniac od indeksu 1 dla wspolczynnikow Fouriera

	for (k=1; k <= liczbaCechMFC; k++) // Petla po wszystkich indeksach cech MFC
	{
		L=1;
		fact = czestotliwoscBazowa;
		for (i= L; i< polowaWierszy; i++, fact+=czestotliwoscBazowa) // P�tla po po�owie wsp�czynnik�w Fouriera
		{
			if (fact >= filtryMEL[k-1])
			{
				poczMEL[k] = i;
				break;
			}
		}
		for (L = i; L<= polowaWierszy; L++, fact+=czestotliwoscBazowa)
		{
			if (fact >= filtryMEL[k+1])
			{
				koniecMEL[k] = L - 1;
				break;
			}
		}
		if (L > polowaWierszy)
			koniecMEL[k] = polowaWierszy;

		// Wspolczynniki wzmocnienia w trojkatnym filtrze dla nale��cych do niego wsp. Fouriera
		for ( M = poczMEL[k]; M <=koniecMEL[k]; M++)
		{
			ftmp = M * czestotliwoscBazowa;
			if ( ftmp <= filtryMEL[k] )
                wspolczMEL[k][M] = (ftmp - filtryMEL[k-1])/ (filtryMEL[k] - filtryMEL[k-1]);
			else
				wspolczMEL[k][M] = (filtryMEL[k+1] - ftmp)/ (filtryMEL[k+1] - filtryMEL[k]);
		}

	}
#ifdef MONITOR
#ifdef MYVISUAL
	CString tmpstr, info;
	i=0;

	info.Format("Filtry MEL: ind->czest,pocz,koniec,waga[pocz],waga[kon]\n");
		for (k=0; k<= liczbaCechMFC; k++)
		{
			tmpstr.Format("%d\t-> %f, %d, %d, %f, %f\n", k, filtryMEL[k], poczMEL[k], koniecMEL[k], wspolczMEL[k][poczMEL[k]], wspolczMEL[k][koniecMEL[k]]);
			info += tmpstr;
		}
		AfxMessageBox(info);

#endif
#endif

	// Koniec przygotowania danych dla MFC
	//////////////////////////////////////////



	// M=16; // ?
	stop=1e-5;
	Smin=0.000001;

	// Alokuj tablice dla uczenia klasyfikatora
	UstawTablice();

}


void CROSMDoc::UstawTablice() // Alokuj tablice dla cech klas
{
	int i;
	long rozmiar_spektra = kolumnyUproszczonego * wierszeUproszczonego; //

	for (i=0; i< MAKS_LA_KLAS; i++)
	{
		u[i]=new double [rozmiar_spektra];
		s[i]=new double [rozmiar_spektra];
		p[i]=new double [rozmiar_spektra];
	}

	for (i=0; i< MAKS_LA_KLAS; i++)
	{
		for (long j=0; j< rozmiar_spektra; j++)
			u[i][j]=s[i][j]= p[i][j] =0.0;
	};
}


void CROSMDoc::UsunTablice() // Usun tablice dla cech klas
{

	for (int i=0; i< MAKS_LA_KLAS; i++)  // WK
	{
			delete  u[i] ;
			delete  s[i] ;
			delete  p[i] ;
	}
}

void CROSMDoc::InicjujAnalize(int bezPrzerw)
{
	long i;
	//CString info; //WINDOWS

	// 1) Znajdz warto�� maksymaln�
	double aktualnaWart, minimalnaWart, sredniaWart, licznik;

	maxWartosc = 0.0;
	for(i=0; i<liczba_probek; i++)
	{ aktualnaWart = fabs(probki[i]);
	  if (aktualnaWart > maxWartosc) maxWartosc = aktualnaWart;
	}

#ifdef MONITOR
	if (bezPrzerw == 0)
	{
			info.Format("A_max wynosi: %.2f\n", maxWartosc);
			AfxMessageBox(info);
	}
#endif

	// 2) Ewentualnie przeskaluj "niezerowy" sygnal do zakresu <-16k, 16k>
	if (maxWartosc > poziomCiszy)
	{
		licznik = 16000.0 / maxWartosc;
		for (i=0; i<liczba_probek; i++)
			probki[i] = probki[i] * licznik;
		maxWartosc = 16000.0;
	}

	// Pomocniczy bufor
	double *tmpprobki= new double[liczba_probek];

	//////////////////////////////////////////////////////////////
	//3) Filtr gornoprzepustowy
	//maxWartosc = 0.0;
	//minimalnaWart = 0.0;
	//sredniaWart = 0.0;
	//tmpprobki[0] = 0.0;
	//tmpprobki[1] = 0.0;
	//tmpprobki[liczba_probek-1] = 0.0;
	//tmpprobki[liczba_probek-2] = 0.0;

	//for(i=2; i < liczba_probek - 2; i++)
	//{
	//	aktualnaWart = fabs(probki[i+2] - probki[i-2] + 2 * (probki[i+1] - probki[i-1]));
	//	if (aktualnaWart < 1000.0)
	//		tmpprobki[i] = probki[i] * aktualnaWart / 1000.0;
	//	else tmpprobki[i] = probki[i];

	//	if (tmpprobki[i] > maxWartosc)
	//		maxWartosc = tmpprobki[i];
	//	if (tmpprobki[i] < minimalnaWart)
	//		minimalnaWart = tmpprobki[i];
	//	sredniaWart += tmpprobki[i];
//	}

	// 4) Skopiowanie spowrotem polaczone z usunieciem skladowej stalej;
	//	sredniaWart = sredniaWart / liczba_probek;
	//	for(i=0; i < liczba_probek; i++)
	//		probki[i] = tmpprobki[i] - sredniaWart;
	//	maxWartosc -= sredniaWart;
	//	minimalnaWart -= sredniaWart;



	//if (fabs(minimalnaWart) > fabs(maxWartosc))
	//	maxWartosc = fabs(minimalnaWart);

	// 2a) Ponownie przeskaluj sygnal do zakresu <-16000, 16000> po zmianie
	//if (maxWartosc > poziomCiszy)
	//{
	//	licznik = 16000.0 / maxWartosc;
	//	for (i=0; i<liczba_probek; i++)
	//		probki[i] = probki[i] * licznik;
	//	maxWartosc = 16000.0;
	//}
	//////////////////////////////////

#ifdef MONITOR
	if (maxWartosc <= poziomCiszy)
	{
		if (bezPrzerw == 0)
		{
			info.Format("Sama cisza - max wynosi: %.2f\n", maxWartosc);
			AfxMessageBox(info);
		}
	}
#endif


	// 4) FILTR PREEMFAZY
	for (i=0; i<liczba_probek; i++) // Podpamietaj aktualny sygna�
		tmpprobki[i] = probki[i];
	for (i=1; i<liczba_probek; i++) // Wykonaj przekszta�cenie na aktualnych danych
		probki[i] = probki[i] - tmpprobki[i-1] * WSPOLCZ_PREEMFAZY;

	// 4a) Ponownie przeskaluj sygnal do zakresu <-16000, 16000> po preemfazie
	if (maxWartosc > poziomCiszy) // pod warunkiem, �e nie by�a to cisza
	{
		maxWartosc = 0.0; // Ponownie znajdziemy now� maksymaln� warto��
		for(i=0; i<liczba_probek; i++)
		{
			aktualnaWart = fabs(probki[i]);
			if (aktualnaWart > maxWartosc)
				maxWartosc = aktualnaWart;
		}
		// Teraz mo�emy przeskalowa� amplitud� pr�bek
		licznik = 16000.0 / maxWartosc;
		for (i=0; i<liczba_probek; i++)
			probki[i] = probki[i] * licznik;
        maxWartosc = 16000.0;
	}

	// 5) Oblicz energi� zaktualizowanych probek sygnalu
	// Energia b�dzie potrzebna dla detekcji 2 ramek sygna�u u�ytecznego
	delete [] probkiEnergia; // Najpierw usu� poprzedni bufor dla energii probek
	probkiEnergia = new double[liczba_probek];
	for (i=0; i<liczba_probek; i++)
		probkiEnergia[i] = probki[i] * probki[i];

	//
	delete [] tmpprobki;
	waska_ramka = 0;

	// kolumnyUproszczonego = kolumnyUproszcz; // ?

	otwarty=true;

}

long CROSMDoc::ZnajdzRamke(int szeroki) // Detekcja ramki pojedynczej lub podw�jnej
{
	long cisza_poczatkowa = 0;
	long cisza_koncowa = 0;
	long liczbaProbekBezCiszy;
	long i, j, lPotrzebnychProbek, polowaLPP;
	int lPotrzebnychOkien, polowaLPO;
//	double cisza = maxWartosc * ( poziomCiszy / 100);
//	double suma_testowa;

	if  (szeroki == 1) // Ramka o podwojnej szerokosci
	{
		liczbaProbekBezCiszy = 2 * kolumnyUproszcz * okno; //
		lPotrzebnychOkien = 2 * kolumnyUproszcz;
		lPotrzebnychProbek = (2 * kolumnyUproszcz - 1) * odstepOkna + okno;
		// W celu znalezienia energii koncentrujemy sie na polowkowej ramce
		polowaLPP = lPotrzebnychProbek /  2; // koncentrujemy sie na polowie ramki
		polowaLPO = kolumnyUproszcz;
	}
	else // Docelowa "waska" ramka
	{
		liczbaProbekBezCiszy = kolumnyUproszcz * okno; //
		lPotrzebnychOkien = kolumnyUproszcz;
		lPotrzebnychProbek = (kolumnyUproszcz - 1) * odstepOkna + okno;
		// Teraz w celu znalezienia energii koncentrujemy si� na ca�ej ramce
		polowaLPP = lPotrzebnychProbek ;
		polowaLPO = kolumnyUproszcz ;
	}

	// Zeruj ramke
	for (i=0; i< liczba_probekBezCiszy;i++) // Zeruj ramke dla probek
		probkiBezCiszy[i] = 0;

	double energia = 0.0;
	double maks_energia;

	// Sprawdz, czy dostepna jest wystarczajaco duza licza probek
	if (polowaLPP >= liczba_probek) // Za malo probek, nie ma po\trzeby szukac ramki w sygnale
	{
		cisza_poczatkowa = 0;
	}
	else
	{
		long ostatni_test = liczba_probek - polowaLPP;

		for (j=0; j < polowaLPP; j++)
			energia += probkiEnergia[j];

		cisza_poczatkowa = 0;
		maks_energia = energia;
		for ( i=1; i < ostatni_test; i++)
		{
			energia -= probkiEnergia[i-1];
			energia += probkiEnergia[i + polowaLPP];
			if (energia > maks_energia)
			{
				maks_energia = energia;
				cisza_poczatkowa = i;
			}
		}
	}

	// Normalizuj wed�ug energii ramki
	double skaluj;
	if (maks_energia >= 1) // Niezerowa energia
	{
		skaluj = maks_energia / polowaLPP; // �rednia energia na 1 okno
		skaluj = NORMALNA_ENERGIA / sqrt(skaluj); // �rednia amplituda normalizowana do 1000
		for (i=0; i < liczba_probek; i++) // Normalizuj wszystkie pr�bki sygna�u
			probki[i] *= skaluj;
	}
	// Przygotuj dane w ramce
	long l = cisza_poczatkowa;

	long k0, k;

	// Przekopiuj srodkowa polowe maks. - polaczone z ewentualnym "zaplataniem okien"
	k0 = polowaLPO / 2;
	k = k0 * okno;
	for (i=0; i < polowaLPO; i++, l+=odstepOkna) //
		for (j=0; j < okno; j++, k++)
			probkiBezCiszy[k] = probki[l + j];

	if  (szeroki == 1) // Ramka o podwojnej szerokosci
	{
		// Dodaj ko�cow� cz�� ramki
		for (i=polowaLPO; i < lPotrzebnychOkien - k0; i++, l+=odstepOkna) //
		{
			for (j=0; j < okno; j++, k++)
				if ((l+j) < liczba_probek)
					probkiBezCiszy[k] = probki[l + j];
				else break;
			if ((l+j) >= liczba_probek)
				break;
		}

		// Dodaj pocz�tkow� cz�� ramki
		k = k0 * okno - 1;
		l = cisza_poczatkowa + odstepOkna - 1;
		for (i=k0 -1; i >=0; i--, l-=odstepOkna)
		{
			for (j=0; j<okno; j++, k--)
				if ((l-j) < 0) break;
				else
					probkiBezCiszy[k] = probki[l-j];

			if ((l-j) < 0) break;
		}

		cisza_poczatkowa = l - okno + 1;
		if (cisza_poczatkowa < 0 )
			cisza_poczatkowa = 0;
	}

// Poprzednia wersja - usuwanie ciszy
	// for ( i=0; i < (liczba_probek - okno); i++)
	//{	suma_testowa = 0.0;
	//	for ( j=0; j < okno; j++)
	//		suma_testowa += fabs(probki[i+j]);
    //    suma_testowa = suma_testowa / okno;
	//	if (suma_testowa > cisza)  break;
	//}
	//cisza_poczatkowa=i;

	//for (i = liczba_probek-1; i>= cisza_poczatkowa + okno; i--)
	//{	suma_testowa = 0.0;
	//	for ( j=0; j < okno; j++)
	//		suma_testowa += fabs(probki[i-j]);
    //    suma_testowa = suma_testowa / okno;
	//	if (suma_testowa > cisza)  break;
	//}
	//cisza_koncowa = i;

	//liczba_probekBezCiszy = cisza_koncowa - cisza_poczatkowa + 1;
	//if (liczba_probekBezCiszy > 2 * okno)
	//	liczba_probekBezCiszy = 2 * kolumnyUproszczonego * okno;
	//if (liczba_probekBezCiszy < okno) // Nieudana proba usuniecia ciszy (WK)
	//{ cisza_poczatkowa = liczba_probek/2 - (kolumnyUproszczonego * okno) / 2;
	//  liczba_probekBezCiszy = 2 * kolumnyUproszczonego * okno;
	//}
	//if ((cisza_poczatkowa + liczba_probekBezCiszy) > liczba_probek)
	//	cisza_poczatkowa -= cisza_poczatkowa + liczba_probekBezCiszy - liczba_probek;


	return cisza_poczatkowa;
}



void CROSMDoc::ObliczCechyOkien(int typ_spektro, int bezPrzerw)
{
	//CString info; //WINDOWS
 	double maxWartosc=0;
 	long i = 0;
	int k, j, l;

	double *spekt;	//komorki numerowane od zera, kolumnami od lewej strony
	double *ptr_probki = probkiBezCiszy; // Wejsciowy wektor sygnalu
 	int l_probek = liczba_probekBezCiszy;

 	int liczba_wierszy = wierszeUproszczonego; // Rozmiar docelowej tablicy cech
 	int liczba_kolumn = kolumnyUproszczonego * 2; // 2 ramki zawierajace cechy dla pewnej liczby okien

	unsigned long rozmiar_szeroki; // Rozmiar spektrogramu
	if (typ_spektro==1)
		rozmiar_szeroki = okno * kolumnyUproszcz * 2; // Szeroka wersja
	else
		rozmiar_szeroki = okno * kolumnyUproszcz ; // Waska wersja - w zasadzie juz jej nie stosuje

	unsigned long rozmiar_prostego = liczba_kolumn * liczba_wierszy;
	// Rozmiar tablicy cech - spektrogram po redukcji rozdzielczosci

////////////////////////
#ifdef MONITOR
// Testuje algorytmy fft
//	int lprobek = 8;
//	int mokno = 8;
//	double ptrprobki[] = {10.0, 20.0, 30.0, 10.0, 0.0, -10.0, -20.0, -30.0 };
//	double wFun[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

//	double *newSTFT= stft(ptrprobki, lprobek, mokno, wFun); // Short time Fourier Transform
//	if (lprobek % mokno > 0)
//		lprobek += (mokno - lprobek%mokno);

 //	spekt = spect(newSTFT, lprobek, mokno);
//	delete [] newSTFT;

//	CString tstr;
//	info.Format("Test fft 8\n");
//	for (j=0; j<8 ; j++)
//	{
//		tstr.Format("%f; ", spekt[j]);
//		info += tstr;
//	}
//	AfxMessageBox(info);
#endif
/////////////////////////


	// 1) Oblicz FFT dla wszystkich okien podw�jnej ramki
	double *newstft= stft(ptr_probki, l_probek, okno, windowFun); // short time Fourier transform
	if (l_probek % okno > 0)
		l_probek += (okno - l_probek%okno);

	// 2) Oblicz widmo amplitudowe (poddane pierwiastkowaniu lub logarytmowaniu)
 	spekt = spect(newstft, l_probek, okno); // spectrum
	delete [] newstft; // WK (zwolnienie danych zaalokowanych wczesniej w funkcji stft()

	for (i=0; i< rozmiar_szeroki; i++) // To kopiowanie zrobione po to aby nie zmieniac procedury spect
		spektAszer[i] = spekt[i];

	delete [] spekt; // Zwolnienie danych zaalokowanych w funkcji spect()

	//
	if (bezPrzerw == 0) // przerywaj prac� w celu wizualizacji wynik�w cz�ciowych
	{
		typSpektrogramu =2; // Poka� szeroki spektrogram: spektAszer
		/*
		UpdateAllViews(NULL);
		if (MONITOR)
			AfxMessageBox( "Obliczono spektrogram. "); */ //WINDOWS
	}
	//


	// 3) Przeskaluj zgodnie z MEL skal� - w praktyce redukcja rozdzielczosci spektrogramu
	// redukcjaSpektrogramu(spektAszer, spektA, rozmiar_szeroki / okno, okno, liczba_kolumn, liczba_wierszy);
	SkalujWedlugMEL(spektAszer, spektA, rozmiar_szeroki / okno, okno, liczba_kolumn, liczba_wierszy,
		liczbaCechMFC, poczMEL, koniecMEL, wspolczMEL);

	//
	if (bezPrzerw == 0) // przerywaj prac� w celu wizualizacji wynik�w cz�ciowych
	{
		typSpektrogramu = 1;// Poka� w�sk� tablic� cech: spektA
		/*
		UpdateAllViews(NULL);
		if (MONITOR)
			AfxMessageBox( "Obliczono cechy MFC. ");*/ //WINDOWS
	}
	//

	//if (NORMALIZUJ_SPEKTROGRAM == 1) // W��czane/wy��czane w ROSMDoc
	//{
	//	//maxWartosc = maxWartosc / 100; // Dla unikniecia b. malych wartosci
	//	if (maxWartosc > 0.0)
	//		i = 0;
	//		for (k=0; k< liczba_kolumn; k++)
	//		{
	//			spektA[i] = spektA[i] / maxWartosc; // normalizuj zerowy wsp.
	//			i++;
	//			for (j=1; j< liczba_wierszy; j++, i++) // Normalizuj wspolczynniki pozostale
	//			{
	//				spektA[i] = spektA[i] / maxWartosc;
	//			}
	//		}
	//}


	// 4) Oblicz sumaryczn� energi� dla ka�dej kolumny MFC
	// Umiesc te sume w kom�rce ka�dej kolumny o 0-wym indeksie

	i = 0;
	for (k=0; k< liczba_kolumn; k++, i+=liczba_wierszy)
	{
	  // Liczymy sum� wszystkich wsp�czynnikow MFCC.
		spektA[i] = 0.0;
		for (j=1; j<= liczbaCechMFC; j++)
		{
			spektA[i] += spektA[i+j];
		}

		// Normalizuj pozosta�e poza zerowym
		// if (spektA[i] > 0.0)
		//	for (j=1; j<= liczbaCechMFC; j++) // Normalizuj wspolczynniki pozostale
		//	{
		//		spektA[i+j] = spektA[i+j] / spektA[i];
		//	}

#ifdef MONITOR
//		CString tmpstr;
//		info.Format("FC - kolumna %d\n", k);
//		for (j=0; j<=K ; j++)
//		{
  //         tmpstr.Format("%d\t-> %f\n", j, spektA[i+j]);
	//	   info += tmpstr;
//		}
//		AfxMessageBox(info);
#endif
	}

	//

  ////////////////////////
	//LICZ_SPEKTRO = 0; // Teraz juz spektrogram istnieje;

	// 5) Oblicz cechy MFCC
	double *Cepstrum = new double[lCechMFCC];

	double lambda = liczbaCechMFC - 1;
	//double lambda = K - 11;
	double lambda_pol = 0.5 * lambda;

	i = 0; // Indeks elementu w spektrogramie
	for (k=0; k< liczba_kolumn; k++, i+= liczba_wierszy) // Petla po kolumnach
	{
		Cepstrum[0] = spektA[i]; // Pierwsza cecha to energia zerowego wspolczynnika
		for (j=1; j< lCechMFCC ; j++)
		{
			Cepstrum[j] = 0.0;
			for (l=1; l<= liczbaCechMFC; l++)
				Cepstrum[j] += spektA[i+ l] * cos (j * (l - 0.5)* 3.141592653589793/ liczbaCechMFC) ;
		}

	  // 6) Liftracja cech MFCC
	 // for (j=1; j< lCechMFCC ; j++)
	//	  Cepstrum[j] = (1 + lambda_pol * sin (3.141592653589793 * j / lambda)) * Cepstrum[j];

	  // 7) Przepisz cechy cepstralne po liftracji do spektA
		for (j=1; j< lCechMFCC ; j++)
			spektA[i+j] = Cepstrum[j];
	 // Zeruj cechy o indeksie wy�szym ni� lCechMFCC
		for (j= lCechMFCC; j< liczba_wierszy ; j++)
			spektA[i+j] = 0.0;

#ifdef MONITOR
//		CString tmpstr;
//		info.Format("liftrowane MFCC - kolumna %d\n", k);
//		for (j=0; j<=K ; j++)
//		{
//           tmpstr.Format("%d\t-> %f\n", j, spektA[i+j]);
//			info += tmpstr;
//		}
//		AfxMessageBox(info);
#endif
	}
	delete [] Cepstrum;

	// 6) Uzupe�nij o gradienty cech MFCC
	i = 0; // Indeks elementu w spektrogramie
	long p1 = 0;
	long p2 = 0;
	long n1 = liczba_wierszy;
	long n2 = n1 + liczba_wierszy; //
	int lxDwaCechMFCC = lCechMFCC * 2;
	// Dla 0-wej kolumny
	for (l=0, j= lCechMFCC; j< lxDwaCechMFCC; j++, l++)
		spektA[i+j] = (2 * (spektA[n2+l] - spektA[i+l]) + (spektA[n1+l]- spektA[i+l]))/5;
	// Dla 1-giej kolumny
	p1 = 0;
	p2 = 0; // teraz 0-wa
	n1 += liczba_wierszy; // teraz 2-ga
	n2 += liczba_wierszy; // teraz 3-cia
	i += liczba_wierszy;
	for (l=0, j = lCechMFCC; j< lxDwaCechMFCC ; j++, l++)
		spektA[i+j] = (2 * (spektA[n2+l] - spektA[i+l]) + (spektA[n1+l] - spektA[p2+l]) )/6;

	// Dla pozostalych "wewnetrznych" kolumn
	p1 = 0;
	p2 = liczba_wierszy; // teraz 1-a
	n1 += liczba_wierszy; // teraz 3-ci
	n2 += liczba_wierszy; // teraz 4-ta
	i += liczba_wierszy; // teraz 2-ga

	for (k=2; k< liczba_kolumn - 2; k++, i+= liczba_wierszy) // Petla po kolumnach
	{
		for (l=0, j = lCechMFCC; j< lxDwaCechMFCC ; j++, l++)
			spektA[i+j] = (2 * spektA[n2+l] + spektA[n1+l] - spektA[p2+l] -  2 * spektA[p1+l]) /10;
		// inkrementuj indeksy
		p1 += liczba_wierszy;
		p2 += liczba_wierszy; // teraz 1-a
		n1 += liczba_wierszy; // teraz 3-ci
		n2 += liczba_wierszy; // teraz 4-ta
	}

	// Dla przedostatniej kolumny
		for (l=0, j = lCechMFCC; j< lxDwaCechMFCC ; j++, l++)
			spektA[i+j] = (spektA[n1+l] - spektA[p2+l] +  2 * (spektA[i+l] - spektA[p1+l])) /6;
	// Dla ostatniej kolumny - inkrementuj ostatni raz indeksy
		p1 += liczba_wierszy; // teraz ostatnia - 2
		p2 += liczba_wierszy; // teraz ostatnia - 1
		i +=liczba_wierszy; // teraz ostatnia kolumna
		for (l=0, j = lCechMFCC; j< lxDwaCechMFCC; j++, l++)
			spektA[i+j] = (spektA[i+l] - spektA[p2+l] +  2 * (spektA[i+l] - spektA[p1+l])) /5;

// 7) Normalizacja kolumn (wsp�czynnik�w MFCC) wzgl�dem zerowego sk�adnika

    i = 0;
	for (k=0; k< liczba_kolumn; k++)
	{
		maxWartosc = spektA[i];
		if (maxWartosc > 0.0)
		for (j=1; j< lxDwaCechMFCC; j++)
		{
			spektA[i+j] = spektA[i+j] / maxWartosc;
		}
		i+=liczba_wierszy;
	}



}

int CROSMDoc::WykonajKlasyfikacje(double* ymin) // Zwraca indeks zwycieskiej klasy
				// i tablic� jakosci dopasowa� dla wszystkich komend w modelu s��w
{
	// Alternatywne rozpoznawanie:
 	// y = t_gmm(spektA,G,D,L,M,u,s,p);
	// gdzie spektA to uproszczony spektrogram obecnie otwartego pliku
	//	y=t_gmm(spektA,40,32,6,M,u[0],s[0],p[0]);

	//CString info, tmpstr;

	const double WSPOLCZ_KARY = 5000.0; // Czy kara zdominuje?
	//const double WSPOLCZ_KARY = 60.0;
	// const double WSPOLCZ_KARY = 0.1; // Praktycznie bez kary - nie dzia�a dobrze
	double *y = new double[LA_KLAS];
	double *yT = new double[LA_KLAS];
	double *yN = new double[LA_KLAS];
	int *najlepszakolumna = new int[LA_KLAS];

 // double *U= new double[6*M*16];
 // double *S= new double[6*M*16];
 // double *P= new double[6*M];
	int i,j, k;
	int l, dwieKolumnyUproszczonego;

	long rozmiar_spektra = kolumnyUproszczonego * wierszeUproszczonego; //
	int maxWiersze = wierszeUproszczonego; // Sprawdzamy caly spektrogram
 // int maxWiersze = wierszeUproszczonego / 2; // Poprzednio: sprawdzalismy polowe - bez gradientow
	double val_element, val_nastepny;
	long elementSze, elementWas;
 //long rozmiar_ramki = rozmiar_spektra * 2; // Zawiera 2 ramki
	int nastepny =0;
	int noDelays=0;

	dwieKolumnyUproszczonego = 2 * kolumnyUproszczonego ;

 for (k=0; k< LA_KLAS; k++)
 {

	 for (i=0; i<kolumnyUproszczonego; i++) // Przesuwaj waski w "szerokim"
	 {
		 elementSze = i * wierszeUproszczonego;
	//for (k=0; k< LA_KLAS; k++)
		 y[k] = 0.0;
		 elementWas = 0;

	for (j = 0, l = 0; ((j< kolumnyUproszczonego) && (l < (dwieKolumnyUproszczonego - i))); j++, l++)
	//for (j=0; j< kolumnyUproszczonego; j++)
	{
	  if (j== (kolumnyUproszczonego - 1)) // ostatnia kolumna
		  nastepny = 0;
	  else nastepny = wierszeUproszczonego;

	  val_element = spektA[elementSze]; // "Zerowy wspolczynnik"
	  val_nastepny = spektA[elementSze+nastepny]; // Nastepna kolumna


	  //for (k=0; k< LA_KLAS; k++)
	  //{
		  yT[k] = 0.0; yN[k] = 0.0;
	  //}
	  //Dopasowanie - uzyskujemy jakosc dopasowania - im wieksza tym lepsza
	  //for (k=0; k< LA_KLAS; k++)
	  //{
		  yT[k] = dopasowanie1( &spektA[elementSze], &u[k+1][elementWas], lCechMFCC, i, 0 );
		  yN[k] = dopasowanie1( &spektA[elementSze+nastepny], &u[k+1][elementWas], lCechMFCC, i, 0 );
	  //}
	  // koniec dopasowania

//	  for ( k=1; k<= LA_KLAS; k++)
//	  {

	 //   y[k-1] += fabs(val_element - u[k][elementWas])/ p[k][elementWas]; // To sa "duze" wartosci
//	   yT[k-1] += fabs(val_element - u[k][elementWas]); // To sa "duze" wartosci
//	   yN[k-1] += fabs(val_nastepny - u[k][elementWas]); // To sa "duze" wartosci
//	  }
//	  elementSze++;
//	  elementWas++;

//	  val_element = spektA[elementSze]; // "1-szy wspolczynnik"
//	  val_nastepny = spektA[elementSze+nastepny]; // Nastepna kolumna
//	  for ( k=1; k<= LA_KLAS; k++)
//	  {
	//	    y[k-1] += fabs(val_element - u[k][elementWas])/ p[k][elementWas]; // To sa "duze" wartosci, dlatego z mniejsza waga
//	  	   yT[k-1] += fabs(val_element - u[k][elementWas]); // To sa "duze" wartosci, dlatego z mniejsza waga
//	       yN[k-1] += fabs(val_nastepny - u[k][elementWas]); // To sa "duze" wartosci, dl
//	  }
//	  elementSze++;
//	  elementWas++;

//	  for ( l=2; l<= maxWiersze; l++)
//	  {
//		  val_element = spektA[elementSze];
//		  val_nastepny = spektA[elementSze+nastepny]; // Nastepna kolumna
//		  for ( k=1; k<LA_KLAS; k++)
//		  {
		//   y[k-1] +=  fabs(val_element - u[k][elementWas]) / ( p[k][elementWas]);
// 		    yT[k-1] +=  fabs(val_element - u[k][elementWas]);
//			yN[k-1] +=  fabs(val_nastepny - u[k][elementWas]);
//		  }
//   		  elementSze++;
 //  		  elementWas++;
//
	 //  for ( k = 0; k < LA_KLAS; k++)  // Wybierz 1-sze lub drugie dopasowanie
		   if (yT[k] < yN[k])
		   {
			   y[k] +=yN[k];
			 /* Przesuwamy aktualna kolumne o 2 - jesli jeszcze jest miejsce
				if (l < (dwieKolumnyUproszczonego - i - 1)) // nie jest to ostatnia dodawana kolumna
				{
					elementSze += wierszeUproszczonego;
					l++;
				}
				elementSze += wierszeUproszczonego; // Przesuwamy o druga kolumne
				noDelays = 0; // przesunelismy aktualna kolumne
				*/
		   }
		   else
		   {
			   y[k] += yT[k];
			  /*  noDelays++; // stanelismy na aktualnym
			   if (noDelays > 1)
			   {
				   elementSze += wierszeUproszczonego; // wiecej niz 2 razy na 1 aktualnym nie zostaniemy
				   noDelays = 0;
			   }
			   else
				   l--; // Trzeba zmniejszyc bo na koniec zwiekszymy o 1 a nie ma zmiany
               */
		   }


//    } // end for l
		   if (l == (dwieKolumnyUproszczonego - i - 1)) // ostatnia szeroka kolumna
		   {
		 	 l--;
			 elementSze -= wierszeUproszczonego;
		   }
	elementSze+= wierszeUproszczonego;
	elementWas+= wierszeUproszczonego;
    } // end for j

	if (i==0) // Inicjalizuj dopasowanie dla danej klasy
	{
	//	for ( k=0; k< LA_KLAS; k++)
	//	{
			ymin[k] = y[k];
			najlepszakolumna[k] = 0;
	//	}
	 }
	 else
	 {
	//	for ( k=0; k < LA_KLAS; k++)
			if (y[k] > ymin[k])
			{
				 ymin[k] = y[k];
				 najlepszakolumna[k] = i;
			}
	 }

 } // end for i
} // end for k

// Uzupelniamy dopasowanie o kare wynikajaca z pozostawienia waznych okien wewn�trz podwojnej ramki

	yT[0] = suma2Ramek( spektA, wierszeUproszczonego, dwieKolumnyUproszczonego );
	if ( yT[0] < 0.00000000001)
		yT[0] = 0.00000000001;

	for (k = 0; k< LA_KLAS ; k++)
	{
		y[k] = penalty1( spektA, najlepszakolumna[k], wierszeUproszczonego, kolumnyUproszczonego );
		// Teraz w y[] jest energia pozostalej czesci 2 ramek.
		// Ustalamy wzgledna energie pozostawiona bez dopasowania.
		y[k] = WSPOLCZ_KARY * y[k] / yT[0]; // Badamy wsp�czynnik w zakresie <0.1, 10 >

		ymin[k] = ymin[k] / (1 + y[k]); // Nalezy obnizyc jakosc o pozostawion� cz��
	}

// Wyniki obliczen
 // Kto "wygral"?
 // for (k=0; k< LA_KLAS - 1; k++) ymin[k] = ymin[k] / 10000.0;

 double min_wartosc = ymin[0]; // Inicjalizuj wartoscia dopasowania dla klasy 1
 int kto_wygral = 1;
 for (k=1; k < LA_KLAS; k++) // odklasy 2 do LA_KLAS
 {
	 if (ymin[k] > min_wartosc)
	 {
		 min_wartosc = ymin[k];
		 kto_wygral = k+1;
	 }
 }

 // Dla wizualizacji wyniku rozpoznania
 waska_ramka = najlepszakolumna[kto_wygral - 1] * kolumnyUproszcz / kolumnyUproszczonego;
 //kolumnyUproszczonego = kolumnyUproszcz;


// info.Format("Odl: %.1f:%d, %.1f:%d , %.1f:%d, %.1f:%d, %.1f:%d, %.1f:%d. ",ymin[0],najlepszakolumna[0],  ymin[1],najlepszakolumna[1],
//	 ymin[2], najlepszakolumna[2], ymin[3], najlepszakolumna[3], ymin[4], najlepszakolumna[4], ymin[5], najlepszakolumna[5] );
//	AfxMessageBox(info);


	//i = 0;
	//for (j=0; j<kolumnyUproszczonego; j++, i+=wierszeUproszczonego)
	//{ info.Format("%d: [0]%f, [1]%f , [2]%f, [3]%f, [4]%f, [5]%f, [6]%f, [7]%f, [8]%f ",
	//	j,u[kto_wygral][i], u[kto_wygral][i+1], u[kto_wygral][i+2], u[kto_wygral][i+3], u[kto_wygral][i+4],
	//	u[kto_wygral][i+5], u[kto_wygral][i+6], u[kto_wygral][i+7], u[kto_wygral][i+8]);

	//	AfxMessageBox(info);
	//}

// To wywolanie trzeba poprawic
//    y=t_gmm(spektA,40,16,6,M,U,S,P);

	// Ustalmy sume odleglosci

	double suma_odl = 0.0;

	for (k=0; k < LA_KLAS; k++)
	{
	 //if (ymin[k] > 0.000001) ymin[k] = 1.0/ymin[k];
	 //else ymin[k] = 1000000.0;
		suma_odl += ymin[k];
	}

	for (k=0; k< LA_KLAS; k++)
	{
		ymin[k] =  ymin[k] / suma_odl;
	}


	// Przywr�c ustawienia domyslne dla ramki
	// waska_ramka = 0;

	delete [] y;
	delete [] yT;
	delete [] yN;
	delete [] najlepszakolumna;

//delete [] U;
//delete [] S;
//delete [] P;

 return kto_wygral;

}

void CROSMDoc::RozpoznajKomende() //
{
	 double *ymin = new double[LA_KLAS];
	 int zwycieskaKlasa, k, j;


	 zwycieskaKlasa = WykonajKlasyfikacje( ymin );

	 std::cout << "some winner " << zwycieskaKlasa << " with " << ymin[zwycieskaKlasa - 1] << "%" << std::endl;

	 if (ymin[zwycieskaKlasa - 1] > (1.5 / LA_KLAS) )
	 {
		 std::cout<<"Oto slowo Twoje: "<<zwycieskaKlasa<<std::endl; //NazwaKomendy[zwycieskaKlasa]<<endl;
	 }

	 recog_word_id=zwycieskaKlasa;

	// 	strcpy(recog_word ,NazwaKomendy[zwycieskaKlasa]);

	 #ifdef WINDOWS
	  CString slowo, info, tmpstr;

	 slowo = NazwaKomendy[zwycieskaKlasa];

	 UpdateAllViews(NULL);

     if (ymin[zwycieskaKlasa - 1] > (1.5 / LA_KLAS) )
	 {
		 SyntezaOdpowiedzi("widz� rozumiem", "neutral");

  		 info.Format("Rozpoznano s�owo: %s; przy po�o�eniu ramki: %d.", slowo, waska_ramka/2);
	 }

	 else
	 {
		SyntezaOdpowiedzi("nie rozumiem", "neutral");
		info.Format("Nie rozpoznano - b��d jest zbyt du�y!");
	 }


	AfxMessageBox(info);

	for (k=1; k <= LA_KLAS; k+=20)
	{
		info.Format("Wyniki: (s�owo -> prawdopodobienstwo):\n");
		for (j=k; j<k+20; j++)
		{
            tmpstr.Format("%s\t-> %.2f\n", NazwaKomendy[j], ymin[j-1]);
			info += tmpstr;
			if ((j+1) > LA_KLAS)
				break;
		}
		AfxMessageBox(info);
	}
#endif
	delete [] ymin;
}

void CROSMDoc::WczytajKlasy() // Otwarcia pliku z danymi klasyfikatora
{
	long rozmiar_spektra = kolumnyUproszczonego * wierszeUproszczonego;
	int fd;
/*
	   CString name;
		CFileDialog fileDlg(
		TRUE,
		_T("usp"),
		0,
		0,
		_T("Pliki USP (*.usp)|*.usp||")
		);



	   if (fileDlg.DoModal()==IDOK)
	   {
		   CString name= fileDlg.GetFileName();
	   	   CFile f(name, CFile::modeRead);
*/ //WINDOWS
	fd = open("../data/lppt.usp",O_RDWR);
	if (fd <0)
	{
		printf("File with words' classes doesn't exist\n");
	}

	double *buf= new double[3];
	//read( fd, buf, sizeof( buf ) ); //sizeof( buf )=4
	read( fd, buf, 24 );
	//f.Read(buf,24); //WINDOWS
	// M = buf[0];
	Smin =buf[1];
	LA_KLAS = (int) Smin;
	// stop =buf[2];
	delete[] buf;

		   long size = rozmiar_spektra * 3 * LA_KLAS;

		   buf= new double[size];
		   int nBytes= size * 8;
		   //read( fd, buf, sizeof( buf ) );
		   read( fd, buf, nBytes );
		   //f.Read(buf,nBytes); //WINDOWS
//cout <<"ALA "<<LA_KLAS<<" "<<fd<<endl;
		   //CString info;
		   //info.Format("M= %f, X=%f",M[0],X[0]);
		  //AfxMessageBox(info);

		   long k=0;
		   int i;
		   long j;

		   double aaa;

		   for(i=1; i<=LA_KLAS; i++)
		      for(j=0; j< rozmiar_spektra; j++)
			  {
				 u[i][j]=buf[k];
				 k++;
			  };

		   for(i=1; i<=LA_KLAS; i++)
		      for(j=0; j< rozmiar_spektra; j++)
			  {
				 s[i][j]=buf[k];
				 k++;
			  };

		   for(i=1; i<= LA_KLAS; i++)
		      for(j=0; j<rozmiar_spektra; j++)
			  {
				 p[i][j]=buf[k];
				 k++;
			  };

		   // Na koncu nazwy klas
		   char chbuf[16];

		   for(i=1; i<= LA_KLAS; i++)
		   {
		   		 //read( fd, chbuf, sizeof( chbuf ) );
		   		 read( fd, chbuf, 16 );
			   //f.Read(chbuf, 16); //WINDOWS
			   //NazwaKlasy[i] = chbuf; //WINDOWS
				std::cout << chbuf <<std::endl;
		   }

		   for(i=1; i<= LA_KLAS; i++)
		   {
			    //read( fd, chbuf, sizeof( chbuf ) );
			    read( fd, chbuf, 16 );
			   //f.Read(chbuf, 16); //WINDOWS
			   //NazwaKomendy[i] = chbuf; //WINDOWS
				std::cout << chbuf <<std::endl;
		   }

			close( fd );
		   //f.Close(); //WINDOWS

	   	for (i=0; i<= LA_KLAS; i++)  // to jest liczba klas
		{
			mianownikiSpekt[i]= 3; // Sztucznie zalozona waga 3 dla wczytanych
			for (j=0; j< rozmiar_spektra; j++)
			{
				spektrogramySrednie[i][j] = u[i][j];
				licznikiSpekt[i][j] = 3 * u[i][j];
			}
		}

	/*   }
	   else return;*/ //WINDOWS

}


