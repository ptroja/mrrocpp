//#include "stdafx.h" //WINDOWS

//#include <math.h> //WINDOWS

//#include "monitor.h" //WINDOWS

#define SWAP(a,b) tempr=(a);(a)=(b);(b)=tempr


//////////////////////////////////////////////////////
void four1(double data[], unsigned long nn, int isign)
/*Zastepuje data[1..2*nn] przez nn-punktowa FFT (isgn=1) lub przez nn-punktowa IFFT (isign=-1)
data jest tablica nn liczb zespolonych, nn musi byc potega 2  (nie sprawdzane!).
Czesci rzeczywiste i zespolone na przemian*/
{
    unsigned long n,mmax,m,j,istep,i;
    double wtemp,wr,wpr,wpi,wi,theta;
    double tempr,tempi;
    n=nn << 1;
    j=1;
    for (i=1;i<n;i+=2) {
        if (j > i) {
            SWAP(data[j],data[i]);
            SWAP(data[j+1],data[i+1]);
        }
        m=n >> 1;
        while (m >= 2 && j > m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }


    mmax=2;
    while (n > mmax) {
        istep=mmax << 1;
        theta=isign*(6.28318530717959/mmax);
        wtemp=sin(0.5*theta);
        wpr = -2.0*wtemp*wtemp;
        wpi=sin(theta);
        wr=1.0;
        wi=0.0;
        for (m=1;m<mmax;m+=2) {
            for (i=m;i<=n;i+=istep) {
                j=i+mmax;
                tempr=wr*data[j]-wi*data[j+1];
                tempi=wr*data[j+1]+wi*data[j];
                data[j]=data[i]-tempr;
                data[j+1]=data[i+1]-tempi;
                data[i] += tempr;
                data[i+1] += tempi;
            }
            wr=(wtemp=wr)*wpr-wi*wpi+wr;
            wi=wi*wpr+wtemp*wpi+wi;
        }
        mmax=istep;
    }
}

/////////////////////////////////////////////////////
void realft(double data[], unsigned long n, int isign)

/*Oblicza FFT zestawu n punktow o wartosciach rzeczywistych, zawartych w data. Zastepuje je przez
polowe transformaty dla dodatnich czestotliwosci (na podstawie symetrii mozna odtworzyc czesc dla
ujemnych). Ostatni element transformaty (rzeczywisty) upchniety w czesci urojonej pierwszego.
n musi byc potega dwojki, dla isign=-1 oblicznana jest IFFT, ale wyniki musza byc pomnozone przez 2/n*/
{

    unsigned long i,i1,i2,i3,i4,np3;
    double c1=0.5,c2,h1r,h1i,h2r,h2i;
    double wr,wi,wpr,wpi,wtemp,theta;
    theta=M_PI/(double)(n>>1);
    if (isign == 1) {
        c2 = -0.5;
        four1(data,n>>1,1);
    } else {
        c2=0.5;
        theta = -theta;
    }
    wtemp=sin(0.5*theta);
    wpr = -2.0*wtemp*wtemp;
    wpi=sin(theta);
    wr=1.0+wpr;
    wi=wpi;
    np3=n+3;
    for (i=2;i<=(n>>2);i++) {
        i4=1+(i3=np3-(i2=1+(i1=i+i-1)));
        h1r=c1*(data[i1]+data[i3]);
        h1i=c1*(data[i2]-data[i4]);
        h2r = -c2*(data[i2]+data[i4]);
        h2i=c2*(data[i1]-data[i3]);
        data[i1]=h1r+wr*h2r-wi*h2i;
        data[i2]=h1i+wr*h2i+wi*h2r;
        data[i3]=h1r-wr*h2r+wi*h2i;
        data[i4] = -h1i+wr*h2i+wi*h2r;
        wr=(wtemp=wr)*wpr-wi*wpi+wr;
        wi=wi*wpr+wtemp*wpi+wi;
    }
    if (isign == 1) {
        data[1] = (h1r=data[1])+data[2];
        data[2] = h1r-data[2];
    } else {
        data[1]=c1*((h1r=data[1])+data[2]);
        data[2]=c1*(h1r-data[2]);
        four1(data,n>>1,-1);
    }
}


////////////////////////////////////////////////////////////////
// funkcja stft() : oblicza FFT dla kolejnych okien o szerokosci w sygnalu o dlugosci n
double* stft(double* data, unsigned long n, unsigned long w, double *window)
{
/* Oblicza Short Term Fourier Transform zestawu n punktow o wartosciach rzeczywistych. Zastepuje je przez
polowe transformaty dla dodatnich czestotliwosci. w (szerokosc okna) musi byc potega 2 i wieksze lub rowne n.
Dane sa uzupelniane w razie potrzeby zerami, tak aby n' bylo wielokrotnoscia w. Okna nie zachodza na siebie*/

    int i, j, rest;
	double* newdata;

    rest=(int)n % (int)w;

	if (rest>0) {
		newdata = new double[n+w-rest];
	} else {
		newdata = new double[n];  // Uwaga: tu jest alokacja pami�ci, jej zwolnienie musi by� zapewnione po stronie wywo�ania (WK)
	}

    for (i=0; i<(int)n; i++) {
		newdata[i] = data[i];
    }
	if (rest>0) {
		for (i=(int)n; i<(int)n+(int)w-rest; i++) {
			newdata[i]=0;
		}
	}
	// Liczba okien w sygnale mowy
    int turns=n/w;
	if (rest>0) turns++;

	// Wywo�anie RealFT dla 1 okna
	long indeks = 0;
    for (i=0; i<turns; i++, indeks+=w) {
		for (j=0; j<w; j++)   // Funkcja okna Hamminga
			newdata[indeks + j] *= window[j];

		realft((newdata + indeks - 1 ),w,1); // Uwaga: w realft() ineksowanie newdata poczawszy od 1 (WK)
    }

	return newdata;
}

///////////////////////////////////////////////////////////////////////
double* spect(double* data, unsigned long data_size, unsigned long n)
/* Oblicza widmo amplitudowe sygnalu na podstawie STFT. Zwraca tablice zawierajaca zarowno ujemne
jak i dodatnie czestotliwosci, bez maksymalnej, dzieki czemu zachowuje ten sam rozmiar co STFT (n).
(nie zawiera nic dla max. czestotliwosci gdyz wynosi ono tyle co dla min. (ujemnej) czestotliowosci)
*/
{   const double MINLOGAR = 0.0;
	double fiRe, fiIm;
	int i, j;
	int count = data_size / n; // Liczba kolumn (okien)
	if (data_size % n > 0) count++; // Ostatnie okno jest niepelne

	double* specdata = new double[data_size]; // Alokacja ale brak zwolnienia (WK)

	for (i=0; i<count; i++) // Dla kazdej kolumny (okna)
	{
		fiRe = data[i*n+1];
		fiRe = fiRe * fiRe;
		specdata[i*n + (n/2)] = sqrt(MINLOGAR + fiRe); // dla fmin = fmax jest real, skala log

		for (j=2; j<(int)n; j+=2) {
			fiRe = data[(i+1)*n-j];
			fiIm = data[(i+1)*n-j+1];
			specdata[i*n+ n/2 + j/2]= sqrt(MINLOGAR + fiRe * fiRe + fiIm * fiIm); // log (WK)
		}

        fiRe = data[i*n];
		fiRe = fiRe * fiRe;
		specdata[i*n]= sqrt(MINLOGAR + fiRe);   //f0 , skala log (WK)

		for (j=2; j<(int)n; j+=2) {
			fiRe = data[i*n+j];
			fiIm = data[i*n+j+1];
			specdata[i*n + j/2] = sqrt(MINLOGAR + fiRe * fiRe + fiIm * fiIm ); // log (WK)
		}

	// W specdata dane ulozone sa teraz kolumnami dla poszczegolnych okien nastepujaco:
	// [|F0| , |F1| , ... , |Fmax-1|, |F max = F min |, |Fmin+1| , |Fmin+2| , ... |F-1|  ] dla N=128, 128 wsp. (WK)
	// teraz wystarczy odrzucic druga czesc po |Fmax|
	}

	return specdata;
}



/////////////////////////////////////////////////////////////
void specta(double* tab_we, double tab_wy[], int szer_we, int dl_we, int szer_wy, int dl_wy)
/* Dokonuje redukcji rozdzielczo�ci (polowy+1) widma spektrogramu danego tablic�
  tab_we o rozmiarach szer_we (ilosc kolumn)i dl_we (ilosc wierszy)
  na tablice wyjsciowa
  tab_wy o rozmiarach szer_wy (ilosc kolumn), dl_wy (ilosc wierszy).
  Metoda usredniania z nachodzeniem na siebie usrednianych blokow. */
{
	int dx,dy;
	int l=0;
	int v=0;
	int k=0;
	int i, j, t, u;
	long rozmiar_tablicy = szer_wy * dl_wy;

	for (long i=0; i< rozmiar_tablicy; i++)
		tab_wy[i]=0;

	// Cz�sci ca�kowite poni�szych dziele�.
	dx=(int)floor((double)szer_we / szer_wy); // Relacja liczby kolumn
	dy=(int)floor((double)dl_we / dl_wy); // Relacja liczby wierszy

	// "Zerowy" wiersz pozostaje bez zmian
	l=0; // Indeks wypelnianych kolumn
	for(i=0; i < dx * szer_wy; i+=dx)
	{
		k=0; // Indeks wierszy w ramach aktualnej kolumny
		v=0;
		j=0;
		int ogranicz_x = i + dx;
		if (szer_we < ogranicz_x)
			ogranicz_x = szer_we;

		for(t = i; t < ogranicz_x; t++)
		{
			u = j;
			tab_wy[ l * dl_wy + k] = tab_we[t * dl_we + u]; // Uwaga: zmieni�em z += na =
			v++;
		}
	//	tab_wy[l * dl_wy + k] /= v;
		k++;
		v=0;

		l++;
	}

	l=0; // Indeks wypelnianych kolumn
	for(i=0; i < dx * szer_wy; i+=dx)
	{

		k=1; // Indeks wierszy w ramach aktualnej kolumny
		for(j=1; j< dy * dl_wy; j+=dy)
		{
		int ogranicz_x = i + dx;
		if (szer_we < ogranicz_x) ogranicz_x = szer_we;

		for(int t = i; t < ogranicz_x; t++)
		{
			int ogranicz_y = j + dy;
			if (dl_we < ogranicz_y) ogranicz_y = dl_we;

			for(u = j; u < ogranicz_y; u++)
			{
				tab_wy[ l * dl_wy + k] = tab_we[t * dl_we + u]; // Uwaga zmiana z += na =
				v++;
			}
		}
//		tab_wy[l * dl_wy + k] /= v;
		k++;
		v=0;
		}
		l++;
	}

}

////////////

void redukcjaSpektrogramu(double* tab_we, double tab_wy[], int szer_we, int dl_we, int szer_wy, int dl_wy)
/* Nowa, alternatywna metoda redukcji spektrogramu droga wybierania podelementow
  Redukcja spektrogramu danego tablic� tab_we o rozmiarze:
  szer_we (ilosc kolumn) x dl_we (ilosc wierszy)
  na tablice wyjsciowa  tab_wy o rozmiarze:
  szer_wy (ilosc kolumn) x dl_wy (ilosc wierszy).
  */
{

	long rozmiarWy = szer_wy * dl_wy;
	long iWe, iWy;
	int j, jWe, dX, dY;

	dX = szer_we / szer_wy;
	dY = dl_we / dl_wy;

	iWe = 0;
	for (iWy = 0; iWy< rozmiarWy; iWy += dl_wy)
	{
		tab_wy[iWy] = tab_we[iWe]; // zerowy element kolumny jest kopiowany

		jWe = 1;
		for (j = 1; j< dl_wy; j++) // w ramach kolumny
		{
			tab_wy[iWy + j] = tab_we[iWe + j * dY];
		}
		iWe += dl_we * dX;
	}
}
////////////

void SkalujWedlugMEL(double *tab_we, double* tab_wy, int szer_we, int dl_we, int szer_wy, int dl_wy,
					 int liczbaCech, int* poczMEL, int* koniecMEL, double **wspolczMEL)
{
	long rozmiarWy = szer_wy * dl_wy;
	long iWe, iWy;
	int j, k, jWe, dX, dY;
	double wspolczynnik;

	dX = szer_we / szer_wy; // relacja liczby kolumn (szerokosci okien)
	dY = dl_we / dl_wy; // relacja liczby wierszy (dlugosci kolumn)

	iWe = 0;
	for (iWy = 0; iWy< rozmiarWy; iWy += dl_wy)
	{
		tab_wy[iWy] = tab_we[iWe]; // zerowy element kolumny jest kopiowany

		jWe = 1;
		for (j = 1; j<= liczbaCech; j++) // w ramach kolumny
		{
			tab_wy[iWy + j] = 0.0;

			// obliczenie k-tego wspolczynnika MFC
			for (k = poczMEL[j]; k<= koniecMEL[j]; k++)
			{
				tab_wy[iWy + j] += wspolczMEL[j][k] * tab_we[iWe + k];
			}
		}
		for (j = liczbaCech + 1; j< dl_wy; j++) // w ramach kolumny - gorna czesc - reszte zerujemy
			tab_wy[iWy + j] = 0.0;

		iWe += dl_we * dX;
	}

}

//////////////////
