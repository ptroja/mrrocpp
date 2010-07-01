
#ifndef __EDP_SPEAKER_TTS_H
#define __EDP_SPEAKER_TTS_H


#define	MAXOUTDATA	1000000
#define averagef0	118;

class Phoneme {
	public:
		std::string sName;	// instead of setting directly, use name() - that way cCategory and bVoiced will be set too
		signed char cCategory;
		bool bVoiced;
		bool bSoft;
		signed char cSyllableFromWordEnd;
		signed char cHalfSyllable;
		signed char cPhraseBoundary;
		signed char cStress;

		Phoneme();	// default constructor

		void name(std::string);	// set sName, cCategory and bVoiced
};

class PhonemePlus: public Phoneme {
	// expand Phoneme class by adding more variables
	public:
		unsigned int iBegSec;
		unsigned int iBegMSec;
		unsigned int iEndSec;
		unsigned int iEndMSec;	// remember that .001234 is stored as 1234!!!
		unsigned int iLength;
		//TODO: zmienic nazwy na ucF0start i ucF0end
		unsigned char iF0start;	// first fundamental frequency sample
		unsigned char iF0end;	// last fundamental frequency sample

		PhonemePlus();	// default constructor
};

class Unit {
	public:
		std::vector<unsigned int> viPhonemePlus;
		float fTargetCost;
		short int iType;	// -1 - # or B, 0 - phoneme, 1 - syllable, 2 - unit

		Unit();	// default constructor

		void clear();
};

unsigned long SayIt(char *pszInputText, char *pszEmotionType,short int *piBuffSpeechOut);
class Segment {
	public:
		double dTimeBeg;
		double dTimeEnd;
};

//main:
std::vector<unsigned char> fnReadF0(std::string sTextFile);

//NLP:
std::vector<Phoneme> PerformNLPAnalysis(std::string sText);

void AddPhoneme (const char *chPhonemeName, std::vector<Phoneme> *vphoPhonemes);

//select:

std::vector<PhonemePlus> fnReadCorpus(std::string sTextFile, std::string sLabFile, std::vector<unsigned char>& vcF0); //reads in corpus.lab file

std::vector<Unit> fnTextIntoSyllables(const std::vector<Phoneme>& vphoText); //performs syllabification

float fnTargetCost(PhonemePlus pCandidate, Phoneme pText); //calculates target cost

float fnUnitTargetCost(Unit uCandidate, Unit uText, std::vector<PhonemePlus>& vphoCorpus, std::vector<Phoneme>& vphoText); //calucalates unit's target cost

float fnConcatenationCost(Unit uPrevious, Unit uCurrent, std::vector<PhonemePlus>& vphoCorpus); // calculates concatenation cost

std::vector<unsigned int> fnFindBestPath(unsigned int iUnit, unsigned int iWindowSize, std::vector< std::vector<Unit> >& vCandidates, std::vector<PhonemePlus>& vphoCorpus); //finds the best Viterbi path

std::vector<Unit> fnMergeUnits(const std::vector< std::vector<Unit> >& vvuVector); //merges units, if adjacent in the corpus

std::vector<Segment> fnGenerateSegments (std::vector<Unit>& vuVector, std::vector<PhonemePlus>& vphoCorpus); //generates vector of segments to concatenates

std::vector<Segment> SelectUnits(std::vector<Phoneme> vphoText, std::vector<unsigned char>& vcF0); //performs unit selection

//concat:
std::vector<short int> ConcatenateUnits(std::vector<Segment> vsegUnits, std::vector<unsigned char>& vF0);

// by Y
void init_buffers_from_files();


#endif
