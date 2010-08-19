#include <string>
#include <cstring>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>	// unitsel - setw, setfill
#include <fstream>	// unitsel
#include <vector>	// untisel [Standard Template Library]
#include <cmath>	// unitsel - ceil(), floor()
#include <ctime>	// unitsel - clock()
#include <cstdlib>
#include <cstdio>
//#include <windows.h> //WIN

#include "robot/speaker/tts.h" //QNX
typedef unsigned char BYTE; //QNX
typedef unsigned long DWORD; //QNX


using namespace std;

// DEFAULT WEIGHTS
float config_fTargetCostWeight = static_cast <float> (0.5); // concatenation cost weight = 1 - target cost weight
// NOTE: dla 0.1 (wazniejsza konkatenacja niz cel) popular[n'e] brane jest z popular[n'e]jszy, dla 0.5 i 0.9 bardziej pasuje do konca wyrazu
float config_fMaxTargetCostPerPhoneme = static_cast <float> (0.5); // divide units with TC per phoneme higher then this value into phonemes

// DEFAULT TARGET COST PARAMETERS
float config_TC_fSyllableMismatchPenalty = static_cast <float> (0.1);
float config_TC_fEndingSyllableMismatchPenalty = static_cast <float> (0.2);
float config_TC_fPhraseBoundaryNearMismatchPenalty = static_cast <float> (0.2);
float config_TC_fPhraseBoundaryFarMismatchPenalty = static_cast <float> (0.6);

// DEFAULT CONCATENATION COST PARAMETERS
float config_CC_fSamePhonemes = static_cast <float> (1.2);
float config_CC_fSamePhoneticCategory = static_cast <float> (1.4);
float config_CC_fSameVoice = static_cast <float> (2.0);
float config_CC_fAllTheRest = static_cast <float> (2.0);
float config_CC_fHighNeighbourInfluencePenalty = static_cast <float> (10.0);
float config_CC_fSilenceNeighbourMismatchPenalty = static_cast <float> (10.0);
float config_CC_fF0Difference_0to10 = static_cast <float> (0.0);
float config_CC_fF0Difference_10to20 = static_cast <float> (0.5);
float config_CC_fF0Difference_20to30 = static_cast <float> (1.0);
float config_CC_fF0Difference_30more = static_cast <float> (1.0);

// OTHER PARAMETERS
bool config_bDisplayF0Calculation = false;
float config_fF0SamplingOffset = static_cast <float> (0.00375); // where did we start to measure the F0 with the sampling rate config_fF0Sampling
float config_fF0Sampling = static_cast <float> (0.010); // in seconds (10 ms)
float config_fPreEliminationThreshold = static_cast <float> (0.1);

// by Y
vector <PhonemePlus> vphoCorpus;
vector <unsigned char> vcF0;

FILE *input, *output;
float win_size = 0;
float win_size1 = 0;
double ts, tk;
unsigned int i_win_size1 = 0;
unsigned int i_win_size2 = 0;
bool detect_space = false;
bool file_output = false;
float period1, period2;
float f0; //data from corpus.f0
float window1;

unsigned int z, n, i;
short *WAVE_buffer1;

int t_corr;
float t_dev;
unsigned long tk_new, ts_new;
double result, maximum;

unsigned long start_byte, end_byte;
unsigned long new_data_size = 0;
unsigned long N = 0;

BYTE data[5];
BYTE Format[5];
BYTE ChunkID_fmt[5];
BYTE ChunkID[5];
//prepare buffer for holding wave data
short *WAVE_buffer;

//prepare variables for wave file header data
DWORD WAVE_size;
short WAVE_format_tag, WAVE_channels, WAVE_block_align, WAVE_bits_per_sample;
DWORD WAVE_format_length, WAVE_sample_rate, WAVE_av_byte_per_sec, WAVE_data_size;

Phoneme::Phoneme() :
	sName(""), cCategory(0), bVoiced(false), cSyllableFromWordEnd(0), cHalfSyllable(0), cPhraseBoundary(0), cStress(0)
{
	/* this is the faster way to set default values
	 variables are set during initialization instead of
	 being initialized and then set to default value (using copy operation) */
}

void Phoneme::name(string sString)
{
	sName = sString;
	if (sName == "o" || sName == "e" || sName == "a" || sName == "i" || sName == "I" || sName == "u") {
		cCategory = 1; // samogloska
		bVoiced = true;
	} else if (sName == "p" || sName == "t" || sName == "k" || sName == "k'") {
		cCategory = 2; // wybuchowe UV
		bVoiced = false;
	} else if (sName == "b" || sName == "d" || sName == "g" || sName == "g'") {
		cCategory = 3; // wybychowe V
		bVoiced = true;
	} else if (sName == "f" || sName == "s" || sName == "s'" || sName == "x" || sName == "S") {
		cCategory = 4; // trace UV
		bVoiced = false;
	} else if (sName == "ts" || sName == "Ts" || sName == "tS") {
		cCategory = 4; // zwarto-trace UV
		bVoiced = false;
	} else if (sName == "v" || sName == "z" || sName == "z'" || sName == "h" || sName == "Z") {
		cCategory = 5; // trace V
		bVoiced = true;
	} else if (sName == "dz" || sName == "Dz" || sName == "dZ") {
		cCategory = 5; // zwarto-trace V
		bVoiced = true;
	} else if (sName == "n" || sName == "n'" || sName == "m" || sName == "N") {
		cCategory = 6; // nosowe
		bVoiced = true;
	} else if (sName == "l" || sName == "w") {
		cCategory = 7; // boczne
		bVoiced = true;
	} else if (sName == "j") {
		cCategory = 7; // aproksymaty
		bVoiced = true;
	} else if (sName == "r") {
		cCategory = 8; // drzace
		bVoiced = true;
	} else if (sName == ".") { //specjalne:
		sName = "#";
		cCategory = 101; // kropka
		bVoiced = false;
	} else if (sName == ",") {
		sName = "#";
		cCategory = 102; // przecinek
		bVoiced = false;
	} else if (sName == ";") {
		sName = "#";
		cCategory = 103; // �rednik
		bVoiced = false;
	} else if (sName == "?") {
		sName = "#";
		cCategory = 104; // znak zapytania
		bVoiced = false;
	} else if (sName == "|") {
		cCategory = 110; // word boundary
		bVoiced = false;
	} else {
		cCategory = 0;
		bVoiced = false;
	}
	if (sName == "k'" || sName == "g'" || sName == "s'" || sName == "z'" || sName == "Ts" || sName == "Dz" || sName
			== "n'" || sName == "j")
		bSoft = true;
	else
		bSoft = false;
}

PhonemePlus::PhonemePlus() :
	iBegSec(0), iBegMSec(0), iEndSec(0), iEndMSec(0), iLength(0), iF0start(0), iF0end(0)
{
	/* this is the faster way to set default values
	 variables are set during initialization instead of
	 being initialized and then set to default value (using copy operation) */
}

Unit::Unit() :
	fTargetCost(0), iType(0)
{
	/* this is the faster way to set default values
	 variables are set during initialization instead of
	 being initialized and then set to default value (using copy operation) */
}

void Unit::clear()
{
	fTargetCost = 0;
	iType = 0;
	viPhonemePlus.clear();
	return;
}

vector <unsigned char> fnReadF0(string sTextFile)
{
	vector <unsigned char> vcVector;
	fstream pTextFile;
	string sLine;

	pTextFile.open(sTextFile.c_str(), ios::in);

	if (pTextFile.is_open()) {
		while (!pTextFile.eof()) {
			getline(pTextFile, sLine);

			// read value of fundamental frequency
			vcVector.push_back(atoi(sLine.substr(0, sLine.find('.')).c_str()));
		}
		pTextFile.close();
	} else {
		cout << "error! | Can't open file " << sTextFile << ".";
		exit(1);
	}
	return vcVector;
}

// by Y
void init_buffers_from_files()
{
	vcF0 = fnReadF0("../data/corpus.f0"); //reading in F0 info
	vphoCorpus = fnReadCorpus("../data/corpus.nlp", "../data/corpus.lab", vcF0);

	input = fopen("../data/corpus.wav", "rb");
	if (input == NULL) /*Check source file for open/access errors*/
	{
		printf("ERROR: Corpus file is missing !! check if corpus.wav is in current directory\n");
		system("PAUSE");
		exit(1);
	}

	//set to output wave file
	//file_output=true;

	if (input) {
		/*
		 BYTE data[5];
		 BYTE Format[5];
		 BYTE ChunkID_fmt[5];
		 BYTE ChunkID[5];
		 //prepare buffer for holding wave data
		 short *WAVE_buffer;
		 data[4]='\0';
		 Format[4]='\0';//
		 ChunkID_fmt[4]='\0';//
		 ChunkID[4]='\0';//
		 //prepare variables for wave file header data
		 DWORD WAVE_size;
		 short WAVE_format_tag, WAVE_channels, WAVE_block_align, WAVE_bits_per_sample;
		 DWORD WAVE_format_length, WAVE_sample_rate, WAVE_av_byte_per_sec, WAVE_data_size;
		 */

		data[4] = '\0';
		Format[4] = '\0';//
		ChunkID_fmt[4] = '\0';//
		ChunkID[4] = '\0';//

		fread(ChunkID, sizeof(BYTE), 4, input); //read in first four bytes

		if (!std::strcmp((char *) ChunkID, "RIFF")) {
			fread(&WAVE_size, sizeof(DWORD), 1, input);

			fread(Format, sizeof(BYTE), 4, input);

			if (!std::strcmp((char *) Format, "WAVE")) {
				fread(ChunkID_fmt, sizeof(BYTE), 4, input);

				fread(&WAVE_format_length, sizeof(DWORD), 1, input);

				fread(&WAVE_format_tag, sizeof(short), 1, input);
				fread(&WAVE_channels, sizeof(short), 1, input);
				fread(&WAVE_sample_rate, sizeof(DWORD), 1, input);
				fread(&WAVE_av_byte_per_sec, sizeof(DWORD), 1, input);
				fread(&WAVE_block_align, sizeof(short), 1, input);
				fread(&WAVE_bits_per_sample, sizeof(short), 1, input);
				fread(data, sizeof(BYTE), 4, input);

				fread(&WAVE_data_size, sizeof(DWORD), 1, input);

				WAVE_buffer = (short *) malloc(sizeof(short) * (WAVE_data_size / WAVE_block_align)); //set buffer for data
				fread(WAVE_buffer, sizeof(short), WAVE_data_size / WAVE_block_align, input); //read in our whole sound data chunk

			}
		}
		fclose(input);
	}

}
;

void AddPhoneme(const char *chPhonemeName, vector <Phoneme> *vphoPhonemes)
{
	Phoneme phoPhoneme;
	string sPhonemeName = string(chPhonemeName);
	phoPhoneme.name(sPhonemeName);
	vphoPhonemes->push_back(phoPhoneme);
}

vector <Phoneme> PerformNLPAnalysis(string sText)
{
	vector <Phoneme> vphoPhonemes;
	/*
	 char chCharIn, chCharInL, chCharInR, chCharInR2, chLeftVoiced, chRightVoiced, pszBuff[4];
	 string::size_type stI;
	 Phoneme phoPhoneme;
	 string sPhonemeName;


	 //zamiana na lowercase
	 for (stI = 0; stI < sText.length(); stI++) {
	 chCharIn = sText.at(stI);

	 switch (chCharIn)
	 {
	 case 'A':
	 chCharIn = '�';
	 break;
	 case 'C':
	 chCharIn = '�';
	 break;
	 case 'E':
	 chCharIn = '�';
	 break;
	 case 'L':
	 chCharIn = '�';
	 break;
	 case 'N':
	 chCharIn = '�';
	 break;
	 case 'O':
	 chCharIn = '�';
	 break;
	 case 'S':
	 chCharIn = '�';
	 break;
	 case 'X':
	 chCharIn = '�';
	 break;
	 case 'Z':
	 chCharIn = '�';
	 break;
	 default:

	 //} //by MS polish diactric signs as upper case

	 if ((chCharIn > 64) && (chCharIn < 91))
	 chCharIn += 32;
	 else
	 switch (chCharIn)
	 {
	 case '�':
	 chCharIn = '�';
	 break;
	 case '�':
	 chCharIn = '�';
	 break;
	 case '�':
	 chCharIn = '�';
	 break;
	 case '�':
	 chCharIn = '�';
	 break;
	 case '�':
	 chCharIn = '�';
	 break;
	 case '�':
	 chCharIn = '�';
	 break;
	 case '�':
	 chCharIn = '�';
	 break;
	 case '�':
	 chCharIn = '�';
	 break;
	 case '�':
	 chCharIn = '�';
	 break;
	 }
	 break;
	 }
	 sText.at(stI) = chCharIn;
	 }

	 //	cout << sText << endl;

	 AddPhoneme("#", &vphoPhonemes); //cisza na pocz�tek

	 for (stI = 0; stI < sText.length(); stI++) {
	 chCharIn = sText.at(stI);

	 if (stI < sText.length() - 1)
	 chCharInR = sText.at(stI + 1); //kontekst prawy
	 else
	 chCharInR = 0; // aby uniknac niezainicjalizowanej wartosci
	 //	if (chCharInR == ' ')
	 //		 if (stI < sText.length()-1)
	 //		  chCharInR = sText.at(stI+2); //je�li spacja, to jeden znak dalej

	 chCharInR2 = 0; //czy samog�oska 2 znaki dalej
	 if (stI < sText.length() - 2) {
	 if ((sText.at(stI + 2) == 'a') || (sText.at(stI + 2) == 'e') || (sText.at(stI + 2) == 'o') || (sText.at(stI
	 + 2) == 'u') || (sText.at(stI + 2) == '�') || (sText.at(stI + 2) == '�') || (sText.at(stI + 2)
	 == 'i') || (sText.at(stI + 2) == 'y'))
	 chCharInR2 = 1;
	 if ((sText.at(stI + 2) == 'i'))
	 chCharInR2 = 2;
	 }

	 chLeftVoiced = 0;
	 if (stI > 0) {
	 chCharInL = sText.at(stI - 1); //kontekst lewy
	 if ((chCharInL == 'a') || (chCharInL == 'b') || (chCharInL == 'd') || (chCharInL == 'e') || (chCharInL
	 == 'g') || (chCharInL == 'i') || (chCharInL == 'j') || (chCharInL == 'l') || (chCharInL == '�')
	 || (chCharInL == 'm') || (chCharInL == 'n') || (chCharInL == 'o') || (chCharInL == '�')
	 || (chCharInL == 'r') || (chCharInL == 'u') || (chCharInL == 'w') || (chCharInL == 'y')
	 || (chCharInL == 'z') || (chCharInL == '�') || (chCharInL == '�') || (chCharInL == '�')
	 || (chCharInL == '�'))
	 chLeftVoiced = 1;
	 } else
	 chCharInL = ' ';

	 //czy d�wi�czny prawy kontekst
	 chRightVoiced = 0;
	 if ((chCharInR == 'a') || (chCharInR == 'b') || (chCharInR == 'd') || (chCharInR == 'e') || (chCharInR == 'g')
	 || (chCharInR == 'i') || (chCharInR == 'j') || (chCharInR == 'l') || (chCharInR == '�') || (chCharInR
	 == 'm') || (chCharInR == 'n') || (chCharInR == 'o') || (chCharInR == '�') || (chCharInR == 'r')
	 || (chCharInR == 'u') || (chCharInR == 'w') || (chCharInR == 'y') || (chCharInR == 'z') || (chCharInR
	 == '�') || (chCharInR == '�') || (chCharInR == '�') || (chCharInR == '�') || (chCharInR == '.'))
	 chRightVoiced = 1;

	 switch (chCharIn)
	 //zasady wymowy litera po literze
	 {
	 case '�':
	 AddPhoneme("o", &vphoPhonemes);
	 if ((chCharInR == 'c') || (chCharInR == 't') || (chCharInR == 'd'))
	 AddPhoneme("n", &vphoPhonemes);
	 else if (chCharInR == '�') //+ �ci!
	 AddPhoneme("n'", &vphoPhonemes);
	 else
	 AddPhoneme("N", &vphoPhonemes);
	 break;

	 case '�':
	 AddPhoneme("e", &vphoPhonemes);
	 if ((chCharInR == 'c') || (chCharInR == 't') || (chCharInR == 'd'))
	 AddPhoneme("n", &vphoPhonemes);
	 else if (chCharInR == '�') //+ �ci!
	 AddPhoneme("n'", &vphoPhonemes);
	 else if (!((chCharInR == 'l') || (chCharInR == '�') || (chCharInR == ' ') || (chCharInR == '.')
	 || (chCharInR == ',') || (chCharInR == '?')))
	 AddPhoneme("N", &vphoPhonemes);
	 break;

	 case 'i':
	 if ((chCharInR == 'a') || (chCharInR == 'e') || (chCharInR == 'o') || (chCharInR == 'u') || (chCharInR
	 == '�') || (chCharInR == '�'))
	 AddPhoneme("j", &vphoPhonemes);
	 else
	 AddPhoneme("i", &vphoPhonemes);
	 break;

	 case 'c':
	 if (chCharInR == 'i') {
	 AddPhoneme("Ts", &vphoPhonemes);
	 if (chCharInR2)
	 stI++;
	 } else if (chCharInR == 'z') {
	 AddPhoneme("tS", &vphoPhonemes);
	 stI++;
	 } else if (chCharInR == 'h') {
	 AddPhoneme("x", &vphoPhonemes);
	 stI++;
	 } else
	 AddPhoneme("ts", &vphoPhonemes);
	 break;

	 case 'd':
	 if (chCharInR == 'z') {
	 stI++;
	 if (chCharInR2 != 2) {
	 if (chRightVoiced)
	 AddPhoneme("dz", &vphoPhonemes);
	 else
	 AddPhoneme("ts", &vphoPhonemes);
	 } else {
	 AddPhoneme("Dz", &vphoPhonemes);
	 if ((sText.at(stI + 2) == 'a') || (sText.at(stI + 2) == 'e') || (sText.at(stI + 2) == 'o')
	 || (sText.at(stI + 2) == 'u') || (sText.at(stI + 2) == '�') || (sText.at(stI + 2)
	 == '�') || (sText.at(stI + 2) == 'i') || (sText.at(stI + 2) == 'y'))
	 stI++;
	 }
	 } else if (chCharInR == '�') {
	 if (chRightVoiced)
	 AddPhoneme("dZ", &vphoPhonemes);
	 else
	 AddPhoneme("tS", &vphoPhonemes);
	 stI++;
	 } else if (chCharInR == '�') {
	 if (chRightVoiced)
	 AddPhoneme("Dz", &vphoPhonemes);
	 else
	 AddPhoneme("Ts", &vphoPhonemes);
	 stI++;
	 } else {
	 if (chRightVoiced)
	 AddPhoneme("d", &vphoPhonemes);
	 else
	 AddPhoneme("t", &vphoPhonemes);
	 }
	 break;

	 case 'n':
	 if (chCharInR == 'i') {
	 AddPhoneme("n'", &vphoPhonemes);
	 if (chCharInR2)
	 stI++;
	 } else if ((chCharInR == 'k') || (chCharInR == 'g'))
	 AddPhoneme("N", &vphoPhonemes);
	 else
	 AddPhoneme("n", &vphoPhonemes);
	 break;

	 case '�':
	 AddPhoneme("n'", &vphoPhonemes); //MS
	 break;

	 case 'k':
	 if (chCharInR == 'i') {
	 AddPhoneme("k'", &vphoPhonemes);
	 if (chCharInR2)
	 stI++;
	 } else
	 AddPhoneme("k", &vphoPhonemes);
	 break;

	 case 'g':
	 if (chCharInR == 'i') {
	 AddPhoneme("g'", &vphoPhonemes);
	 if (chCharInR2)
	 stI++;
	 } else
	 AddPhoneme("g", &vphoPhonemes);
	 break;

	 case 'r':
	 if (chCharInR == 'z') {
	 if ((chLeftVoiced) && (chRightVoiced))
	 AddPhoneme("Z", &vphoPhonemes);
	 else
	 AddPhoneme("S", &vphoPhonemes);
	 stI++;
	 } else
	 AddPhoneme("r", &vphoPhonemes);
	 break;

	 case 's':
	 if (chCharInR == 'i') {
	 AddPhoneme("s'", &vphoPhonemes);
	 if (chCharInR2)
	 stI++;
	 } else if (chCharInR == 'z') {
	 AddPhoneme("S", &vphoPhonemes);
	 stI++;
	 } else
	 AddPhoneme("s", &vphoPhonemes);
	 break;

	 case 'w':
	 if (chRightVoiced)
	 AddPhoneme("v", &vphoPhonemes);
	 else
	 AddPhoneme("f", &vphoPhonemes);

	 break;

	 case 'y':
	 AddPhoneme("I", &vphoPhonemes);
	 break;
	 case '�':
	 AddPhoneme("Ts", &vphoPhonemes);
	 break;
	 case '�':
	 AddPhoneme("w", &vphoPhonemes);
	 break;
	 case '�':
	 AddPhoneme("u", &vphoPhonemes);
	 break;
	 case '�':
	 AddPhoneme("s'", &vphoPhonemes);
	 break;

	 case '�':
	 if (chRightVoiced)
	 AddPhoneme("Z", &vphoPhonemes);
	 else
	 AddPhoneme("S", &vphoPhonemes);
	 break;

	 case ' ':
	 AddPhoneme("|", &vphoPhonemes);
	 break;

	 default:
	 sprintf(pszBuff, "%c", chCharIn);
	 AddPhoneme(pszBuff, &vphoPhonemes);
	 }

	 }

	 //	AddPhoneme("#",&vphoPhonemes);		//cisza na koniec, na wszelki wypadek


	 //phrase boundary syllables
	 signed char cBoundaryType = -1;
	 for (stI = vphoPhonemes.size(); stI > 0; stI--) {
	 phoPhoneme = vphoPhonemes.at(stI - (string::size_type) 1);
	 sPhonemeName = phoPhoneme.sName;
	 if (phoPhoneme.cCategory == 101)
	 cBoundaryType = 0;
	 if (phoPhoneme.cCategory == 1 && cBoundaryType != -1)
	 cBoundaryType++;
	 if ((cBoundaryType > 2) || ((cBoundaryType == 2) && (phoPhoneme.cCategory == 110)))
	 cBoundaryType = -1;
	 phoPhoneme.cPhraseBoundary = cBoundaryType;
	 vphoPhonemes.at(stI - (string::size_type) 1) = phoPhoneme;
	 }

	 for (stI = vphoPhonemes.size(); stI > 0; stI--) {
	 phoPhoneme = vphoPhonemes.at(stI - (string::size_type) 1);
	 if (phoPhoneme.cPhraseBoundary == -1) {
	 if (phoPhoneme.cCategory == 102)
	 cBoundaryType = 2;
	 if (phoPhoneme.cCategory == 1 && cBoundaryType != -1)
	 cBoundaryType++;
	 if ((cBoundaryType > 4) || ((cBoundaryType == 4) && (phoPhoneme.cCategory == 110))) //gdy drugie
	 cBoundaryType = -1;
	 phoPhoneme.cPhraseBoundary = cBoundaryType;
	 vphoPhonemes.at(stI - (string::size_type) 1) = phoPhoneme;
	 }
	 }

	 for (stI = 0; stI < vphoPhonemes.size(); stI++) //PB = 0 zamiast -1
	 {
	 phoPhoneme = vphoPhonemes.at(stI);
	 if (phoPhoneme.cPhraseBoundary == -1)
	 phoPhoneme.cPhraseBoundary = 0;
	 vphoPhonemes.at(stI) = phoPhoneme;
	 }

	 //sylabifikacja
	 signed char cSyllableNo = 0;
	 signed char cLastPhonemeCategory = 0;
	 for (stI = vphoPhonemes.size(); stI > 0; stI--) {
	 phoPhoneme = vphoPhonemes.at(stI - (string::size_type) 1);
	 sPhonemeName = phoPhoneme.sName;
	 if (phoPhoneme.cCategory == 1)
	 cSyllableNo++;
	 else if (sPhonemeName.compare("|") == 0)
	 cSyllableNo = 0;
	 if (cLastPhonemeCategory != 1) //poprzendni nie samog�oska
	 if ((phoPhoneme.cCategory != 1) && (phoPhoneme.cCategory < 100)) //nie samog�oska, nie znak przest.
	 phoPhoneme.cHalfSyllable = 1; //fonem "po��wkowy"
	 phoPhoneme.cSyllableFromWordEnd = cSyllableNo;
	 cLastPhonemeCategory = phoPhoneme.cCategory;
	 vphoPhonemes.at(stI - (string::size_type) 1) = phoPhoneme;
	 }

	 Phoneme phoPhonemeBefore;
	 cLastPhonemeCategory = 0; //modyfikowanie granic, je�li plo bezd�w
	 for (stI = vphoPhonemes.size(); stI > 1; stI--) {
	 phoPhoneme = vphoPhonemes.at(stI - (string::size_type) 1);
	 phoPhonemeBefore = vphoPhonemes.at(stI - (string::size_type) 2);
	 sPhonemeName = phoPhoneme.sName;
	 if ((phoPhoneme.cHalfSyllable == 1) && (phoPhoneme.cCategory != 2))
	 if ((cLastPhonemeCategory == 2) || (cLastPhonemeCategory == 3)) //poprzendni plo
	 {
	 phoPhoneme.cHalfSyllable = 0;
	 phoPhoneme.cSyllableFromWordEnd++;
	 phoPhoneme.cPhraseBoundary = phoPhonemeBefore.cPhraseBoundary;
	 }
	 cLastPhonemeCategory = phoPhoneme.cCategory;
	 vphoPhonemes.at(stI - (string::size_type) 1) = phoPhoneme;
	 }

	 char cLastPhraseBoundary = 0;
	 for (stI = 1; stI < vphoPhonemes.size(); stI++) //uzupe�nianie numer�w koncowym fonemom sylaby
	 {
	 phoPhoneme = vphoPhonemes.at(stI);
	 sPhonemeName = phoPhoneme.sName;
	 if (phoPhoneme.cSyllableFromWordEnd == 0)
	 if (sPhonemeName.compare("|")) {
	 phoPhoneme.cSyllableFromWordEnd = 1;
	 phoPhoneme.cHalfSyllable = 0;
	 phoPhoneme.cPhraseBoundary = cLastPhraseBoundary;
	 }
	 cLastPhraseBoundary = phoPhoneme.cPhraseBoundary;
	 vphoPhonemes.at(stI) = phoPhoneme;
	 }

	 char cLastSyllableFromWordEnd = 0;
	 for (stI = 1; stI < vphoPhonemes.size(); stI++) //zero dla jednosylabowych
	 {
	 phoPhoneme = vphoPhonemes.at(stI);
	 if (phoPhoneme.cSyllableFromWordEnd == 1)
	 if (cLastSyllableFromWordEnd == 0)
	 phoPhoneme.cSyllableFromWordEnd = 0;
	 cLastSyllableFromWordEnd = phoPhoneme.cSyllableFromWordEnd;
	 if (!phoPhoneme.sName.compare("|"))
	 phoPhoneme.sName = "#";
	 vphoPhonemes.at(stI) = phoPhoneme;
	 }

	 vector <Phoneme>::iterator itI;
	 //	if (0)			//!!!!!!!!
	 for (itI = vphoPhonemes.begin() + 1; itI < vphoPhonemes.end(); itI++) {
	 phoPhoneme = *itI;
	 if (phoPhoneme.cCategory > 109) //usuwanie spacji
	 vphoPhonemes.erase(itI--);
	 }
	 */
	return vphoPhonemes;

}
//koniec funkcji PerformNLPAnalysis


vector <PhonemePlus> fnReadCorpus(string sTextFile, string sLabFile, vector <unsigned char>& vcF0)
{
	vector <PhonemePlus> vphoVector;
	fstream pLabFile;
	fstream pTextFile;
	int iLine;

	pLabFile.open(sLabFile.c_str(), ios::in);
	pTextFile.open(sTextFile.c_str(), ios::in);

	//vector<unsigned char> vcF0 = fnReadF0("corpus.f0");

	iLine = 0;
	if (pLabFile.is_open()) {
		unsigned int iCountF0Errors = 0; // count how many times a voiced phoneme was assigned an F0 value equal to 0 (start or end)
		unsigned int iCountF0ErrorsAfterCorrections = 0;
		while (!pLabFile.eof()) {
			PhonemePlus phoTemp;
			string sLine, sTime;
			string::size_type stFrom, stTo, stDot;

			iLine++;

			getline(pLabFile, sLine);
			if (sLine == "")
				break; // empty line

			// read phoneme start time (.lab)
			stFrom = 0;
			stTo = sLine.find(' ');
			sTime = sLine.substr(stFrom, stTo);
			stDot = sTime.find('.'); // find decimal dot, seperate values and store as integers
			phoTemp.iBegSec = atoi(sTime.substr(0, stDot).c_str());
			phoTemp.iBegMSec = atoi(sTime.substr(stDot + 1).c_str());

			// read phoneme end time (.lab)
			stFrom = stTo + 1;
			stTo = sLine.find(' ', stFrom);
			sTime = sLine.substr(stFrom, stTo - stFrom);
			stDot = sTime.find('.'); // find decimal dot, seperate values and store as integers
			phoTemp.iEndSec = atoi(sTime.substr(0, stDot).c_str());
			phoTemp.iEndMSec = atoi(sTime.substr(stDot + 1).c_str());

			// calculate phoneme length
			phoTemp.iLength = ((phoTemp.iEndSec - phoTemp.iBegSec) * 10000000 + (phoTemp.iEndMSec - phoTemp.iBegMSec));

			// read phoneme name (.lab)
			stFrom = stTo + 1;
			stTo = sLine.find(' ', stFrom);
			phoTemp.name(sLine.substr(stFrom, stTo - stFrom));

			getline(pTextFile, sLine);

			// read phoneme name (.nlp) and compare
			stFrom = 0;
			stTo = sLine.find('\t');
			if (phoTemp.sName != sLine.substr(stFrom, stTo)) {
				cout << "Error in line " << iLine << " of corpus.nlp. PhonemePlus name does not match corpus.lab"
						<< endl;
				exit(1);
			}

			// read more data if present (remember to preserve order!)
			if (stTo != string::npos) {
				stFrom = stTo + 1;
				stTo = sLine.find('\t', stFrom);
				phoTemp.cSyllableFromWordEnd = atoi(sLine.substr(stFrom, stTo - stFrom).c_str());
			} else
				phoTemp.cSyllableFromWordEnd = 0;
			if (stTo != string::npos) {
				stFrom = stTo + 1;
				stTo = sLine.find('\t', stFrom);
				phoTemp.cPhraseBoundary = atoi(sLine.substr(stFrom, stTo - stFrom).c_str());
			} else
				phoTemp.cPhraseBoundary = 0;
			if (stTo != string::npos) {
				stFrom = stTo + 1;
				stTo = sLine.find('\t', stFrom);
				phoTemp.cStress = atoi(sLine.substr(stFrom, stTo - stFrom).c_str());
			} else
				phoTemp.cStress = 0;

			// assign fundamental frequency from the F0 samples vector
			unsigned int iStart = static_cast <unsigned int> (ceil((static_cast <float> (phoTemp.iBegSec)
					+ static_cast <float> (phoTemp.iBegMSec) / 10000000 - config_fF0SamplingOffset)
					/ config_fF0Sampling));
			unsigned int iEnd = static_cast <unsigned int> (floor((static_cast <float> (phoTemp.iEndSec)
					+ static_cast <float> (phoTemp.iEndMSec) / 10000000 - config_fF0SamplingOffset)
					/ config_fF0Sampling));

			// apply F0 corrections
			short int iBuffer;
			short int iMaxBuffer = 2;
			if (phoTemp.bVoiced) {
				// if phoneme is voiced then fundamental frequency should be different than 0
				iBuffer = 0;
				while (vcF0[iStart + iBuffer] == 0 && iBuffer < iMaxBuffer)
					iBuffer++;
				phoTemp.iF0start = vcF0[iStart + iBuffer];

				iBuffer = 0;
				while (vcF0[iEnd - iBuffer] == 0 && iBuffer < iMaxBuffer)
					iBuffer++;
				phoTemp.iF0end = vcF0[iEnd - iBuffer];

			} else {
				// if phoneme is unvoiced then fundamental frequency should be equal 0
				iBuffer = 0;
				while (vcF0[iStart + iBuffer] != 0 && iBuffer < iMaxBuffer)
					iBuffer++;
				phoTemp.iF0start = vcF0[iStart + iBuffer];

				iBuffer = 0;
				while (vcF0[iEnd - iBuffer] != 0 && iBuffer < iMaxBuffer)
					iBuffer++;
				phoTemp.iF0end = vcF0[iEnd - iBuffer];

			}

			if (phoTemp.bVoiced && (vcF0[iStart] == 0 || vcF0[iEnd] == 0))
				iCountF0Errors++;
			if (phoTemp.bVoiced && (phoTemp.iF0start == 0 || phoTemp.iF0end == 0))
				iCountF0ErrorsAfterCorrections++;

			vphoVector.push_back(phoTemp);
		}
		pLabFile.close();
		pTextFile.close();
	} else {
		cout << "NUU module | error! | Can't open file " << sLabFile << " or " << sTextFile << "." << endl;
		exit(1);
	}
	return vphoVector;
}

vector <Unit> fnTextIntoSyllables(const vector <Phoneme>& vphoText)
{
	vector <Unit> vuVector;

	for (unsigned int iText = 0; iText < vphoText.size(); iText++) {
		//	cout << static_cast<int> (vphoText[iText].cHalfSyllable) << " ";

		if (vphoText[iText].sName == "#" || vphoText[iText].sName == "B") {
			// silence and breaths are treated as autonomous syllables/units (type -1)
			Unit uTemp;
			uTemp.iType = -1;
			uTemp.viPhonemePlus.push_back(iText);
			vuVector.push_back(uTemp);
			continue;
		}
		if (!vuVector.empty() && vphoText[vuVector.back().viPhonemePlus.back()].cSyllableFromWordEnd
				== vphoText[iText].cSyllableFromWordEnd && vuVector.back().iType != -1) {
			// continue last syllable
			vuVector.back().viPhonemePlus.push_back(iText);
		} else {
			// create new syllable
			Unit uTemp;
			uTemp.iType = 1;
			uTemp.viPhonemePlus.push_back(iText);
			vuVector.push_back(uTemp);
		}
	}

	return vuVector;
}

float fnTargetCost(PhonemePlus pCandidate, Phoneme pText)
{
	float fCost = 0;

	if (pCandidate.cSyllableFromWordEnd != pText.cSyllableFromWordEnd && pCandidate.cSyllableFromWordEnd
			!= pText.cSyllableFromWordEnd + pText.cHalfSyllable) {
		if (pText.cSyllableFromWordEnd + pCandidate.cSyllableFromWordEnd <= 2)
			fCost = config_TC_fSyllableMismatchPenalty; // 0-1, 0-2, 1-0, 2-0 mismatch
		else if (pText.cSyllableFromWordEnd < 3)
			fCost = config_TC_fEndingSyllableMismatchPenalty;
		else
			fCost = config_TC_fSyllableMismatchPenalty;
	}

	if (pCandidate.cPhraseBoundary != pText.cPhraseBoundary) {
		if ((pCandidate.cPhraseBoundary == 1 && pText.cPhraseBoundary == 2) || (pCandidate.cPhraseBoundary == 2
				&& pText.cPhraseBoundary == 1) ||

		//		(pCandidate.cPhraseBoundary == 2 && pText.cPhraseBoundary == 0) ||	//AJ beg
				//		(pCandidate.cPhraseBoundary == 0 && pText.cPhraseBoundary == 2) ||
				(pCandidate.cPhraseBoundary == 4 && pText.cPhraseBoundary == 0) || (pCandidate.cPhraseBoundary == 0
				&& pText.cPhraseBoundary == 4) || (pCandidate.cPhraseBoundary == 6 && pText.cPhraseBoundary == 0)
				|| (pCandidate.cPhraseBoundary == 0 && pText.cPhraseBoundary == 6) || //AJ end

				(pCandidate.cPhraseBoundary == 3 && pText.cPhraseBoundary == 4) || (pCandidate.cPhraseBoundary == 4
				&& pText.cPhraseBoundary == 3) || (pCandidate.cPhraseBoundary == 5 && pText.cPhraseBoundary == 6)
				|| (pCandidate.cPhraseBoundary == 6 && pText.cPhraseBoundary == 5))
			fCost = config_TC_fPhraseBoundaryNearMismatchPenalty;
		else
			fCost = config_TC_fPhraseBoundaryFarMismatchPenalty;
	}

	return fCost;
}

float fnUnitTargetCost(Unit uCandidate, Unit uText, vector <PhonemePlus>& vphoCorpus, vector <Phoneme>& vphoText)
{
	float fCost = 0;
	// TODO: ew. wprowadzic sprawdzanie czy maja po tyle samo fonemow (w przypadku problemow)

	// phoneme by phoneme
	for (unsigned int iPhonemePlus = 0; iPhonemePlus < uCandidate.viPhonemePlus.size(); iPhonemePlus++) {
		fCost
				+= fnTargetCost(vphoCorpus[uCandidate.viPhonemePlus[iPhonemePlus]], vphoText[uText.viPhonemePlus[iPhonemePlus]]);
	}

	return fCost;
}

float fnConcatenationCost(Unit uPrevious, Unit uCurrent, vector <PhonemePlus>& vphoCorpus)
{
	float fCost = 0;

	if (uPrevious.viPhonemePlus.back() + 1 == uCurrent.viPhonemePlus.front()) {
		// corpus neighbours
		return fCost = 0;
	}
	if (vphoCorpus[uPrevious.viPhonemePlus.back()].sName == "#" && vphoCorpus[uCurrent.viPhonemePlus.front()].cCategory
			== 2) {
		// # + plosive UV
		return fCost = 0;
	}

	// check pitch (fundamental frequency, F0) difference voiced phonemes
	if (vphoCorpus[uPrevious.viPhonemePlus.back()].bVoiced && vphoCorpus[uCurrent.viPhonemePlus.front()].bVoiced) {
		unsigned char ucF0difference = abs(vphoCorpus[uPrevious.viPhonemePlus.back()].iF0end
				- vphoCorpus[uCurrent.viPhonemePlus.front()].iF0start);
		if (ucF0difference <= 10)
			fCost += config_CC_fF0Difference_0to10;
		else if (ucF0difference <= 20)
			fCost += config_CC_fF0Difference_10to20;
		else if (ucF0difference <= 30)
			fCost += config_CC_fF0Difference_20to30;
		else
			fCost += config_CC_fF0Difference_30more;
	}

	const PhonemePlus phoCurrentFirst = vphoCorpus[uCurrent.viPhonemePlus.front()];
	const PhonemePlus phoPreviousLast = vphoCorpus[uPrevious.viPhonemePlus.back()];
	PhonemePlus phoCurrentFirstNeighbour, phoPreviousLastNeighbour;

	// watch out for edge values
	if (uCurrent.viPhonemePlus.front() - 1 > 0)
		phoCurrentFirstNeighbour = vphoCorpus[uCurrent.viPhonemePlus.front() - 1];
	else
		phoCurrentFirstNeighbour.name("#");
	if (uPrevious.viPhonemePlus.back() + 1 < vphoCorpus.size())
		phoPreviousLastNeighbour = vphoCorpus[uPrevious.viPhonemePlus.back() + 1];
	else
		phoCurrentFirstNeighbour.name("#");

	// check both directions
	// right to left
	if (phoCurrentFirst.sName == phoPreviousLastNeighbour.sName) {
		fCost += config_CC_fSamePhonemes; // same phonemes
	} else if (phoCurrentFirst.cCategory == phoPreviousLastNeighbour.cCategory) {
		fCost += config_CC_fSamePhoneticCategory; // same phonetic category
	} else if (phoCurrentFirst.bVoiced == phoPreviousLastNeighbour.bVoiced) {
		fCost += config_CC_fSameVoice; // both voiced / unvoiced
	} else
		fCost += config_CC_fAllTheRest; // the rest

	if (phoCurrentFirst.cCategory == 1 && // samogloska
			(phoCurrentFirstNeighbour.cCategory == 1 || // samogloska
					phoCurrentFirstNeighbour.cCategory == 6 || // nosowe
					phoCurrentFirstNeighbour.cCategory == 7 || // boczne
					phoCurrentFirstNeighbour.cCategory == 8) && // drzace
			phoCurrentFirstNeighbour.sName != phoPreviousLast.sName)
		fCost += config_CC_fHighNeighbourInfluencePenalty;

	if (phoCurrentFirst.cCategory == 1 && // samogloska
			phoPreviousLast.bSoft && !phoCurrentFirstNeighbour.bSoft)
		fCost += config_CC_fHighNeighbourInfluencePenalty;

	if ((phoCurrentFirst.cCategory == 1 && // samogloska
			phoPreviousLast.cCategory == 7) //boczne
			&& (phoCurrentFirstNeighbour.sName != phoPreviousLast.sName))
		fCost += config_CC_fHighNeighbourInfluencePenalty;

	if (phoCurrentFirstNeighbour.sName == "#" && phoPreviousLast.sName != "#")
		fCost += config_CC_fSilenceNeighbourMismatchPenalty;

	// left to right
	if (phoPreviousLast.sName == phoCurrentFirstNeighbour.sName) {
		fCost += config_CC_fSamePhonemes; // same phoneme
	} else if (phoPreviousLast.cCategory == phoCurrentFirstNeighbour.cCategory) {
		fCost += config_CC_fSamePhoneticCategory; // same phonetic category
	} else if (phoPreviousLast.bVoiced == phoCurrentFirstNeighbour.bVoiced) {
		fCost += config_CC_fSameVoice; // both voiced / unvoiced
	} else
		fCost += config_CC_fAllTheRest; // the rest

	if (phoPreviousLast.cCategory == 1 && // samogloska
			(phoPreviousLastNeighbour.cCategory == 1 || // samogloska
					phoPreviousLastNeighbour.cCategory == 6 || // nosowe
					phoPreviousLastNeighbour.cCategory == 7 || // boczne
					phoPreviousLastNeighbour.cCategory == 8) && // drzace
			phoPreviousLastNeighbour.sName != phoCurrentFirst.sName)
		fCost += config_CC_fHighNeighbourInfluencePenalty;

	if ((phoPreviousLastNeighbour.sName == "#" || phoPreviousLastNeighbour.cCategory == 2 || // wybuchowe UV
			phoPreviousLastNeighbour.sName == "ts" || // zwarto-trace UV
			phoPreviousLastNeighbour.sName == "Ts" || // ...
			phoPreviousLastNeighbour.sName == "tS"))
		if ( // ...
		!(phoCurrentFirst.sName == "#" || phoCurrentFirst.cCategory == 2 || // wybuchowe UV
				phoCurrentFirst.sName == "ts" || // zwarto-trace UV
				phoCurrentFirst.sName == "Ts" || // ...
				phoCurrentFirst.sName == "tS")) // ...
			fCost += config_CC_fSilenceNeighbourMismatchPenalty;

	return fCost;
}

vector <unsigned int> fnFindBestPath(unsigned int iUnit, unsigned int iWindowSize, vector <vector <Unit> >& vCandidates, vector <
		PhonemePlus>& vphoCorpus)
{

	vector <unsigned int> viCurrentPath(iWindowSize, 0);
	vector <unsigned int> viBestPath;
	float fTotalCost = 1000;

	bool bSkipFlag = false;

	while (true) {

		short int iDigitToIncrement = static_cast <short int> (iWindowSize) - 1; // start with incrementing the last digit (low-order digit)
		while (iDigitToIncrement >= static_cast <short int> (iWindowSize) - 4) {
			if (viCurrentPath[iDigitToIncrement] == vCandidates[iUnit + iDigitToIncrement].size() - 1) {
				// this 'digit' reached the end of candidates' list
				viCurrentPath[iDigitToIncrement] = 0; // make this 'digit' equal zero
				iDigitToIncrement--; // prepare for incrementing next digit
			} else {
				// increment this 'digit'
				viCurrentPath[iDigitToIncrement]++;
				break;
			}
		}

		if (iDigitToIncrement < static_cast <short int> (iWindowSize) - 4)
			break;

		float fTargetCost = 0;
		float fConcatenationCost = 0;
		float fCurrentPartTotalCost = 0;

		// sum of concatenation costs of pairs of units
		fTargetCost += vCandidates[iUnit + 0][viCurrentPath[0]].fTargetCost;
		for (unsigned int iTemp = 0; iTemp < iWindowSize - 1; iTemp++) {
			fTargetCost += vCandidates[iUnit + iTemp + 1][viCurrentPath[iTemp + 1]].fTargetCost;
			fConcatenationCost
					+= fnConcatenationCost(vCandidates[iUnit + iTemp][viCurrentPath[iTemp]], vCandidates[iUnit + iTemp
							+ 1][viCurrentPath[iTemp + 1]], vphoCorpus);
			fCurrentPartTotalCost = (fTargetCost * config_fTargetCostWeight) + (fConcatenationCost * (1
					- config_fTargetCostWeight));

			if (fCurrentPartTotalCost >= fTotalCost) {
				// skip this subpath
				for (iTemp += 2; iTemp < iWindowSize; iTemp++)
					viCurrentPath[iTemp] = static_cast <unsigned int> (vCandidates[iUnit + iTemp].size()) - 1;
				bSkipFlag = true;
				break;
			}
		}

		if (bSkipFlag == true) {
			bSkipFlag = false;
			continue;
		}

		float fCurrentTotalCost = (fTargetCost * config_fTargetCostWeight) + (fConcatenationCost * (1
				- config_fTargetCostWeight));

		if (fCurrentTotalCost < fTotalCost) {
			// new best path found
			viBestPath = viCurrentPath;
			fTotalCost = fCurrentTotalCost;
		}
	}

	return viBestPath;
}

vector <Unit> fnMergeUnits(const vector <vector <Unit> >& vvuVector)
{
	vector <Unit> vuMerged;

	if (vvuVector.empty()) {
		cout << endl << "NNU module | error! | function fnMergeUnits - provided vuuVector is empty";
		exit(1);
	}

	vuMerged.push_back(vvuVector[0][0]);
	for (unsigned int iUnit = 1; iUnit < vvuVector.size(); iUnit++) {
		unsigned int iLastPhonemePlusFromPreviousUnit = vuMerged.back().viPhonemePlus.back();
		unsigned int iFirstPhonemePlusFromCurrentUnit = vvuVector[iUnit][0].viPhonemePlus.front();
		if (iLastPhonemePlusFromPreviousUnit + 1 == iFirstPhonemePlusFromCurrentUnit) {
			// neighbours
			vuMerged.back().viPhonemePlus.insert(vuMerged.back().viPhonemePlus.end(), vvuVector[iUnit][0].viPhonemePlus.begin(), vvuVector[iUnit][0].viPhonemePlus.end());
			vuMerged.back().fTargetCost = 0;
			vuMerged.back().iType = 2;
		} else {
			// not neighbours
			vuMerged.push_back(vvuVector[iUnit][0]);
		}
	}

	return vuMerged;
}

vector <Segment> fnGenerateSegments(vector <Unit>& vuVector, vector <PhonemePlus>& vphoCorpus)
{
	vector <Segment> vsegUnits;

	for (unsigned int iUnit = 0; iUnit < vuVector.size(); iUnit++) {
		Segment segTemp;
		segTemp.dTimeBeg = static_cast <double> (vphoCorpus[vuVector[iUnit].viPhonemePlus.front()].iBegSec)
				+ static_cast <double> (vphoCorpus[vuVector[iUnit].viPhonemePlus.front()].iBegMSec) / 10000000;
		segTemp.dTimeEnd = static_cast <double> (vphoCorpus[vuVector[iUnit].viPhonemePlus.back()].iEndSec)
				+ static_cast <double> (vphoCorpus[vuVector[iUnit].viPhonemePlus.back()].iEndMSec) / 10000000;
		vsegUnits.push_back(segTemp);

		//cout << endl << segTemp.dTimeBeg << "\t\t" << segTemp.dTimeEnd;
	}

	return vsegUnits;
}

vector <Segment> SelectUnits(vector <Phoneme> vphoText, vector <unsigned char>& vcF0)
{

	//	vphoCorpus = fnReadCorpus("/net/yoyek64/dev/shmem/corpus.nlp", "/net/yoyek64/dev/shmem/corpus.lab", vcF0);

	vector <Unit> vuText = fnTextIntoSyllables(vphoText);

	vector <vector <Unit> > vCandidates;

	// FIND CANDIDATES
	for (unsigned int iUnit = 0; iUnit < vuText.size(); iUnit++) {
		vector <Unit> vuTemp;
		Unit uTemp;

		switch (vuText[iUnit].iType)
		{

			case -1:

				// TODO: change this part - better, clean silence and better breathe (still always take the same)
				uTemp.iType = -1;
				if (vphoText[vuText[iUnit].viPhonemePlus[0]].sName == "B")
					uTemp.viPhonemePlus.push_back(23); // B
				else
					uTemp.viPhonemePlus.push_back(61); // #	//AJ

				vuTemp.push_back(uTemp);
				vCandidates.push_back(vuTemp);
				break;

			case 0:

				//
				// do not consider first and last phoneme from the corpus (just in case)  vphoCorpus.size() - 1
				// TODO: what if first or last unit from text will be treated as phoneme? error when trying to check left/right neighbour
				//

				if (vphoText[vuText[iUnit].viPhonemePlus[0]].sName == "t") {
					uTemp.viPhonemePlus.push_back(66);
					vuTemp.push_back(uTemp);
				} else if (vphoText[vuText[iUnit].viPhonemePlus[0]].sName == "k") {
					uTemp.viPhonemePlus.push_back(481);
					vuTemp.push_back(uTemp);
				} else if (vphoText[vuText[iUnit].viPhonemePlus[0]].sName == "p") {
					uTemp.viPhonemePlus.push_back(486);
					vuTemp.push_back(uTemp);
				} else
					for (unsigned int iCorpus = 1; iCorpus < vphoCorpus.size() - 1; iCorpus++) {
						if (vphoText[vuText[iUnit].viPhonemePlus[0]].sName == vphoCorpus[iCorpus].sName) {
							// check the neighbours. at least one of them should be te same in Text as in Corpus
							// NOTE: this method instantly reduces number of candidates for each phoneme by 60-90%
							//if (vphoText[ vuText[iUnit-1].viPhonemePlus.back() ].sName != vphoCorpus[iCorpus-1].sName && vphoText[ vuText[iUnit+1].viPhonemePlus.front() ].sName != vphoCorpus[iCorpus+1].sName) continue;

							// check the neighbours. at least one of them should be the same category in Text as in Corpus
							if ((iUnit > 0) && (iUnit < vuText.size() - 1)) {
								if (vphoText[vuText[iUnit - 1].viPhonemePlus.back()].cCategory != vphoCorpus[iCorpus
										- 1].cCategory && vphoText[vuText[iUnit + 1].viPhonemePlus.front()].cCategory
										!= vphoCorpus[iCorpus + 1].cCategory)
									continue;
							} else if (iUnit > 0) {
								if (vphoText[vuText[iUnit - 1].viPhonemePlus.back()].cCategory != vphoCorpus[iCorpus
										- 1].cCategory)
									continue;
							} else if (vphoText[vuText[iUnit + 1].viPhonemePlus.front()].cCategory
									!= vphoCorpus[iCorpus + 1].cCategory)
								continue;

							// eliminate phonemes with stress
							if (vphoCorpus[iCorpus].cStress == 1)
								continue;
							if (vphoCorpus[iCorpus].cStress == -1)
								continue; //AJ
							if (vphoCorpus[iCorpus].iLength < 25)
								continue; //AJ

							uTemp.clear();
							uTemp.iType = 0; //phoneme
							uTemp.viPhonemePlus.push_back(iCorpus);

							uTemp.fTargetCost = fnUnitTargetCost(uTemp, vuText[iUnit], vphoCorpus, vphoText);

							// new, experimental pre-elimination
							if (uTemp.fTargetCost > config_fPreEliminationThreshold)
								continue;

							vuTemp.push_back(uTemp);
						}
					}

				if (vuTemp.empty()) {
					for (unsigned int iCorpus = 1; iCorpus < vphoCorpus.size() - 1; iCorpus++) {
						// pre-elimination has failed (all candidates were eliminated) - take all phonemes in the corpus
						if (vphoText[vuText[iUnit].viPhonemePlus[0]].sName == vphoCorpus[iCorpus].sName) {
							if (vphoCorpus[iCorpus].cStress == 1)
								continue; //AJ
							if (vphoCorpus[iCorpus].cStress == -1)
								continue; //AJ
							if (vphoCorpus[iCorpus].iLength < 25)
								continue; //AJ
							// new, experimental pre-elimination


							uTemp.clear();
							uTemp.iType = 0;
							uTemp.viPhonemePlus.push_back(iCorpus);
							uTemp.fTargetCost = fnUnitTargetCost(uTemp, vuText[iUnit], vphoCorpus, vphoText);

							//		if (uTemp.fTargetCost > config_fPreEliminationThreshold) continue; //AJ

							vuTemp.push_back(uTemp);
						}
					}
				}
				if (vuTemp.empty()) {
					// fatal error - unknown phoneme (no candidate found in corpus)
					cout << endl << "NUU module | error | unknown phoneme "
							<< vphoText[vuText[iUnit].viPhonemePlus[0]].sName << " (no candidates found in corpus)";
					exit(1);
				}
				vCandidates.push_back(vuTemp);
				break;

			case 1:

				for (unsigned int iCorpus = 0; iCorpus < vphoCorpus.size(); iCorpus++) {
					for (unsigned int iPhonemePlus = 0; vphoText[vuText[iUnit].viPhonemePlus[iPhonemePlus]].sName
							== vphoCorpus[iCorpus + iPhonemePlus].sName; iPhonemePlus++) {
						if (iCorpus + iPhonemePlus + 1 == vphoCorpus.size())
							break; // end of corpus
						if (vphoCorpus[iCorpus + iPhonemePlus].cStress != 0)
							break; //stressed or weak //AJ					if (vphoCorpus[iCorpus].cSyllableFromWordEnd != vphoCorpus[iCorpus + iPhonemePlus].cSyllableFromWordEnd) break;
						if (iPhonemePlus + 1 == vuText[iUnit].viPhonemePlus.size()) {
							// all phonemes in this unit have been checked (they match) - save candidate
							uTemp.clear();
							uTemp.iType = 1;
							for (unsigned int iTemp = 0; iTemp <= iPhonemePlus; iTemp++)
								uTemp.viPhonemePlus.push_back(iCorpus + iTemp);
							uTemp.fTargetCost = fnUnitTargetCost(uTemp, vuText[iUnit], vphoCorpus, vphoText);
							vuTemp.push_back(uTemp);
							iCorpus += iPhonemePlus;
							break;
						}
					}
				}

				bool bTargetCostToHigh = false;
				if (!vuTemp.empty()) {
					float fMinTargetCostPerPhonemePlus = 1000;
					for (unsigned int iTemp = 0; iTemp < vuTemp.size(); iTemp++)
						if (fMinTargetCostPerPhonemePlus > vuTemp[iTemp].fTargetCost
								/ vuTemp[iTemp].viPhonemePlus.size())
							fMinTargetCostPerPhonemePlus = vuTemp[iTemp].fTargetCost
									/ vuTemp[iTemp].viPhonemePlus.size();
					if (fMinTargetCostPerPhonemePlus > config_fMaxTargetCostPerPhoneme)
						bTargetCostToHigh = true;
					// if there are no candidates with target cost per phoneme lower or equal config_fMaxTargetCostPerPhoneme divide this unit into phonemes
				}

				if (vuTemp.empty() || bTargetCostToHigh == true) {
					//if (vuTemp.empty()) cout << endl << "NUU module | info | no candidates found for unit " << iUnit << " (syllable) - dividing into phonemes";
					//else cout << endl << "NUU module | info | all candidates found for unit " << iUnit << " (syllable) have target cost per phoneme > 1 - dividing into phonemes";

					vector <Unit> vuInsert;
					for (unsigned int iPhonemePlus = 0; iPhonemePlus < vuText[iUnit].viPhonemePlus.size(); iPhonemePlus++) {
						uTemp.clear();
						uTemp.iType = 0;
						// !!! DO NOT ADD TARGET COST - this goes to vphoTEXT
						uTemp.viPhonemePlus.push_back(vuText[iUnit].viPhonemePlus[iPhonemePlus]);
						vuInsert.push_back(uTemp);
					}

					vector <Unit>::iterator vuTextIterator = vuText.begin();
					for (unsigned int iTemp = 0; iTemp < iUnit; iTemp++)
						vuTextIterator++;
					vuText.erase(vuTextIterator);
					vuText.insert(vuTextIterator, vuInsert.begin(), vuInsert.end());

					// unit nr iUnit has been removed. added units start with nr iUnit - reduce iUnit by one to find candidates for the first added unit in next cycle. (otherwis it would be skipped)
					iUnit--;
					break;
				}

				vCandidates.push_back(vuTemp);
				break;
		}
	}

	/*	// DISPLAY ALL UNITS WITH NUMBER OF CANDIDATES (vphoText i vCandidates)
	 cout << endl << endl << "unit\telem.\tcand." << endl;
	 for (unsigned int iUnit = 0 ; iUnit < vCandidates.size() ; iUnit++) {
	 // for each unit...
	 cout << iUnit << "\t";
	 for (unsigned int iPhonemePlus = 0 ; iPhonemePlus < vuText[iUnit].viPhonemePlus.size() ; iPhonemePlus++) {
	 cout << vphoText[vuText[iUnit].viPhonemePlus[iPhonemePlus]].sName;
	 }
	 cout << "\t" << vCandidates[iUnit].size();
	 cout << endl;
	 }*/

	// CHOOSE BEST CONCATENATION PATHS
	bool bInitialization = true; // first set consists of 4 units, saves 1st and 2nd and doesn't increase iUnit
	for (unsigned int iUnit = 0; iUnit < vCandidates.size() - 4;) {
		/*
		 [X][X][?][?][_][_]
		 1  2  3  4  5  6
		 - iUnit points to the first unit
		 - units 1 and 2 are already written down (unless it's the first set)
		 - units 3 and 4 will written down during this cycle
		 - units 5 and 6 are only taken into consideration (unless it's the last set)
		 */

		// set the window size (6 or 5 if there was odd number of units, and this is the last set)
		unsigned int iWindowSize = 6;
		if (bInitialization)
			iWindowSize = 4;
		if (iUnit == vCandidates.size() - 5)
			iWindowSize = 5;

		/*
		 // print the whole set
		 cout << endl << "set of units:";
		 for (unsigned int iTemp = 0 ; iTemp < iWindowSize ; iTemp++) {
		 cout << " ";
		 for (unsigned int iPhonemePlus = 0 ; iPhonemePlus < vuText[iUnit + iTemp].viPhonemePlus.size() ; iPhonemePlus++) cout << vphoText[vuText[iUnit + iTemp].viPhonemePlus[iPhonemePlus]].sName;
		 }

		 unsigned int iNumberOfPaths = 1;
		 for (unsigned int iTemp = 0 ; iTemp < iWindowSize ; iTemp++) iNumberOfPaths *= static_cast<int> ( vCandidates[iUnit + iTemp].size() );
		 cout << "   (paths: " << iNumberOfPaths << ")";
		 */

		//
		vector <unsigned int> viBestPath = fnFindBestPath(iUnit, iWindowSize, vCandidates, vphoCorpus);
		//

		if (bInitialization) {
			// save 1st and 2nd path elements
			for (unsigned int iTemp = 0; iTemp < 2; iTemp++) {
				Unit uTemp;
				uTemp = vCandidates[iUnit + iTemp][viBestPath[iTemp]];
				vCandidates[iUnit + iTemp].clear();
				vCandidates[iUnit + iTemp].push_back(uTemp);
			}
			bInitialization = false;
			continue; // iUnit is not increased
		} else if (iUnit >= vCandidates.size() - 6) {
			// save 3rd, 4th, 5th and 6th (if iWindowSize == 6) path elements
			for (unsigned int iTemp = 2; iTemp < iWindowSize; iTemp++) {
				Unit uTemp;
				uTemp = vCandidates[iUnit + iTemp][viBestPath[iTemp]];
				vCandidates[iUnit + iTemp].clear();
				vCandidates[iUnit + iTemp].push_back(uTemp);
			}
		} else {
			// save 3rd and 4th path elements
			Unit uTemp;
			uTemp = vCandidates[iUnit + 2][viBestPath[2]];
			vCandidates[iUnit + 2].clear();
			vCandidates[iUnit + 2].push_back(uTemp);
			uTemp = vCandidates[iUnit + 3][viBestPath[3]];
			vCandidates[iUnit + 3].clear();
			vCandidates[iUnit + 3].push_back(uTemp);
		}

		iUnit += 2;
	}

	vector <Unit> vuMergedAux = fnMergeUnits(vCandidates); //QNX
	vector <Segment> vsegUnits = fnGenerateSegments(vuMergedAux, vphoCorpus); //QNX
	//	vector<Segment> vsegUnits = fnGenerateSegments(fnMergeUnits(vCandidates), vphoCorpus); //WIN

	return vsegUnits;
}
// end of select


vector <short int> ConcatenateUnits(vector <Segment> vsegUnit, vector <unsigned char>& vF0)
{
	/*
	 FILE *input,*output;
	 float win_size=0;
	 float win_size1=0;
	 double ts,tk;
	 unsigned int i_win_size1=0;
	 unsigned int i_win_size2=0;
	 bool detect_space=false;
	 bool file_output=false;
	 float period1,period2;
	 float f0; //data from corpus.f0
	 float window1;

	 unsigned	int z,n,i;
	 short *WAVE_buffer1;

	 int t_corr;
	 float t_dev;
	 unsigned long tk_new,ts_new;
	 double result,maximum;

	 unsigned long start_byte,end_byte;
	 unsigned long new_data_size=0;
	 unsigned long N=0;
	 */
	vector <short int> voutput;

	win_size = 0;
	win_size1 = 0;

	i_win_size1 = 0;
	i_win_size2 = 0;
	detect_space = false;
	file_output = false;

	new_data_size = 0;
	N = 0;

	/*
	 input = fopen("/net/yoyek64/dev/shmem/corpus.wav","rb");
	 if (input==NULL)					//Check source file for open/access errors
	 {
	 printf("ERROR: Corpus file is missing !! check if corpus.wav is in current directory\n");
	 system("PAUSE");
	 exit(1);
	 }
	 */

	//set to output wave file
	//file_output=true;

	if (input) {
		/*
		 BYTE data[5];
		 BYTE Format[5];
		 BYTE ChunkID_fmt[5];
		 BYTE ChunkID[5];
		 //prepare buffer for holding wave data
		 short *WAVE_buffer;
		 data[4]='\0';
		 Format[4]='\0';//
		 ChunkID_fmt[4]='\0';//
		 ChunkID[4]='\0';//
		 //prepare variables for wave file header data
		 DWORD WAVE_size;
		 short WAVE_format_tag, WAVE_channels, WAVE_block_align, WAVE_bits_per_sample;
		 DWORD WAVE_format_length, WAVE_sample_rate, WAVE_av_byte_per_sec, WAVE_data_size;
		 */

		data[4] = '\0';
		Format[4] = '\0';//
		ChunkID_fmt[4] = '\0';//
		ChunkID[4] = '\0';//

		//				fread(ChunkID, sizeof(BYTE), 4, input); 	//read in first four bytes

		if (!std::strcmp((char *) ChunkID, "RIFF")) {
			/*
			 fread(&WAVE_size, sizeof(DWORD), 1, input);

			 fread(Format, sizeof(BYTE), 4, input);
			 */

			if (!std::strcmp((char *) Format, "WAVE")) {
				/*
				 fread(ChunkID_fmt, sizeof(BYTE), 4, input);

				 fread(&WAVE_format_length, sizeof(DWORD),1,input);

				 fread(&WAVE_format_tag, sizeof(short), 1, input);
				 fread(&WAVE_channels, sizeof(short),1,input);
				 fread(&WAVE_sample_rate, sizeof(DWORD), 1, input);
				 fread(&WAVE_av_byte_per_sec, sizeof(DWORD), 1, input);
				 fread(&WAVE_block_align, sizeof(short), 1, input);
				 fread(&WAVE_bits_per_sample, sizeof(short), 1, input);
				 fread(data, sizeof(BYTE), 4, input);


				 fread(&WAVE_data_size, sizeof(DWORD), 1, input);

				 WAVE_buffer = (short *) malloc (sizeof(short) * (WAVE_data_size/WAVE_block_align)); //set buffer for data
				 fread(WAVE_buffer, sizeof(short), WAVE_data_size/WAVE_block_align, input); //read in our whole sound data chunk
				 */
				WAVE_buffer1 = (short *) malloc(sizeof(short) * (WAVE_data_size / WAVE_block_align)); //set target buffer for data
				memset(WAVE_buffer1, 0, (WAVE_data_size / WAVE_block_align));//initialize buffer


				//vector<unsigned char>vF0=fnReadF0("corpus.f0");

				for (unsigned int iTemp = 0; iTemp < vsegUnit.size(); iTemp++) {
					ts = vsegUnit[iTemp].dTimeBeg;
					tk = vsegUnit[iTemp].dTimeEnd;

					detect_space = false;
					if ((ts > static_cast <double> (1.86)) && (ts < static_cast <double> (1.87)))
						detect_space = true;

					start_byte = WAVE_block_align * (int) ((double) WAVE_sample_rate * ts);
					end_byte = WAVE_block_align * (int) ((double) WAVE_sample_rate * tk);

					if (start_byte > WAVE_data_size) {
						printf("Data outside the corpus");
						system("PAUSE");
						exit(1);
					}

					if (end_byte > WAVE_data_size) {
						printf("Data outside the corpus");
						system("PAUSE");
						exit(1);
					}

					unsigned int ts_sample = (int) (ts * WAVE_sample_rate);
					unsigned int tk_sample = (int) (tk * WAVE_sample_rate);

					long lines = static_cast <long> (ts * 100);
					float t_rounded = static_cast <float> (ts * 100);
					if ((t_rounded - lines) > 0.5)
						lines++;

					long lines2 = static_cast <long> (tk * 100);
					t_rounded = static_cast <float> (tk * 100);
					if ((t_rounded - lines2) > 0.5)
						lines2++;

					f0 = vF0[lines];

					if (f0 < 10.0)
						f0 = averagef0;

					period1 = (1 / f0);
					win_size = 2 * period1;
					win_size = win_size * WAVE_sample_rate;
					i_win_size1 = static_cast <unsigned int> (win_size);
					if (((win_size) - i_win_size1) >= 0.5)
						i_win_size1++;

					t_dev = (period1 * WAVE_sample_rate);

					maximum = 0;
					ts_sample--;

					for (i = ts_sample - static_cast <unsigned int> (0.003 * WAVE_sample_rate); i <= ts_sample
							+ static_cast <unsigned int> (t_dev - 0.003 * WAVE_sample_rate); i = i
							+ static_cast <unsigned int> (0.001 * WAVE_sample_rate)) {
						result = 0.0;
						for (n = 0; n <= i_win_size1 - 1; n++) {
							window1
									= static_cast <float> ((0.53836 - 0.46164 * cos((2 * 3.14 * n) / (i_win_size1 - 1)))
											* WAVE_buffer[i + n]);
							result = result + (window1 * window1);
						}

						if (result > maximum) {
							maximum = result;
							ts_new = i;
						}
					}

					t_corr = (ts_new - ts_sample);
					ts_sample = ts_sample + t_corr;

					unsigned int g = 0;
					float h = 0;
					h = static_cast <float> (((i_win_size1 / 2) + (i_win_size2 / 2)));
					h = h / 2;

					g = static_cast <unsigned int> (h);
					if ((h - g) >= 0.5)
						g++;

					n = 0;

					if (N >= g)
						N = N - g;

					for (z = ts_sample; z < ts_sample + ((i_win_size1 - 1) / 2); z++) //bylo <= //bylo win_size zamiast i_win_size
					{
						float window = static_cast <float> (0.53836 - 0.46164
								* cos(((2 * 3.14 * n) / (i_win_size1 - 1))));

						WAVE_buffer1[N] += static_cast <short> (WAVE_buffer[z] * window);
						N++;
						n++;

					}

					f0 = vF0[lines2];
					if (f0 < 10.0)
						f0 = averagef0; //if f0 is smaller then 10, then estimate the approx f0 value.

					period2 = (1 / f0);
					win_size1 = win_size;
					win_size = 2 * period2;
					win_size = win_size * WAVE_sample_rate;
					i_win_size2 = static_cast <unsigned int> (win_size);
					if (((win_size) - i_win_size2) >= 0.5)
						i_win_size2++;

					tk_sample--;
					t_dev = (period2 * WAVE_sample_rate);
					maximum = 0;

					for (i = tk_sample + static_cast <unsigned int> (0.003 * WAVE_sample_rate); i > tk_sample - t_dev
							+ static_cast <unsigned int> (0.003 * WAVE_sample_rate); i = i
							- static_cast <unsigned int> (0.001 * WAVE_sample_rate)) {
						result = 0;
						for (n = 0; n <= i_win_size2 - 1; n++) {
							window1
									= static_cast <float> ((0.53836 - 0.46164 * cos((2 * 3.14 * n) / (i_win_size2 - 1)))
											* WAVE_buffer[i - n]);
							result = result + (window1 * window1);
						}

						if (result > maximum) {
							maximum = result;
							tk_new = i;
						}
					}
					t_corr = (tk_new - tk_sample);
					tk_sample = tk_sample + t_corr;

					for (unsigned int j = ts_sample + ((i_win_size1 - 1) / 2); j < tk_sample - (i_win_size2 / 2); j++) {
						WAVE_buffer1[N] = WAVE_buffer[j];//copy data to secondary buffer. basing on the sample number;
						N++;
					}

					n = (int) ((i_win_size2 - 1) / 2);

					for (z = tk_sample - ((i_win_size2 - 1) / 2); z < tk_sample; z++) {
						float window =
								static_cast <float> (0.53836 - 0.46164 * cos((2 * 3.14 * n) / (i_win_size2 - 1)));
						WAVE_buffer1[N] = static_cast <short> (WAVE_buffer[z] * window);
						n++;
						N++;
					}

				}
				file_output = false;
				if (file_output == true) {
					output = fopen("output.wav", "wb");
					if (output == NULL) /*Check destination file for open/access errors*/
					{
						printf("ERROR: Output file I/O error !!\n");
						system("PAUSE");
						exit(1);
					}

					fwrite(ChunkID, sizeof(BYTE), 4, output);
					fwrite(&WAVE_size, sizeof(DWORD), 1, output);
					fwrite(Format, sizeof(BYTE), 4, output);
					fwrite(ChunkID_fmt, sizeof(BYTE), 4, output);
					fwrite(&WAVE_format_length, sizeof(DWORD), 1, output);
					fwrite(&WAVE_format_tag, sizeof(short), 1, output);
					fwrite(&WAVE_channels, sizeof(short), 1, output);
					fwrite(&WAVE_sample_rate, sizeof(DWORD), 1, output);
					fwrite(&WAVE_av_byte_per_sec, sizeof(DWORD), 1, output);
					fwrite(&WAVE_block_align, sizeof(short), 1, output);
					fwrite(&WAVE_bits_per_sample, sizeof(short), 1, output);
					fwrite(data, sizeof(BYTE), 4, output);

					long new_data_size = N * WAVE_block_align;//write data after windowing
					fwrite(&new_data_size, sizeof(DWORD), 1, output); //data size must be written before actual data
					fwrite(WAVE_buffer1, sizeof(short), N, output); //save data from buffer to file
					fclose(output);
				}

				for (z = 0; z < N; z++) {
					voutput.push_back(WAVE_buffer1[z]);
				}
				//								free(WAVE_buffer);
				free(WAVE_buffer1);
				//								fclose(input);
			}
		}
	}
	return voutput;
}

unsigned long SayIt(char *pszInputText, char *pszEmotionType, short int *piBuffSpeechOut)
{
	string sText(pszInputText);
	unsigned long uiI, uiJ;

	// vcF0 = fnReadF0("/net/yoyek64/dev/shmem/corpus.f0");	//reading in F0 info
	vector <short int> vsiSpeechOut;

	//3 main synthesis processes
	vector <Phoneme> vphoText = PerformNLPAnalysis(sText);
	vector <Segment> vsegUnits = SelectUnits(vphoText, vcF0);
	vsiSpeechOut = ConcatenateUnits(vsegUnits, vcF0);

	for (uiI = 0, uiJ = 0; uiI < vsiSpeechOut.size(); uiI++)
		*(piBuffSpeechOut + uiJ++) = vsiSpeechOut.at(uiI);

	//returns number of samples
	return uiJ;
}
