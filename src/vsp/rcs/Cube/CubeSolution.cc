/////////////////////////////////////////////////////////
// Name: CubeSolution.cpp
// Implements: CubeSolution
/////////////////////////////////////////////////////////


#include "vsp/rcs/Cube/CubeSolution.h"
#include <string>


const char CubeSolution::DELIM = ',';


////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, EXTEND SIZE OF TABLES
////////////////////////////////////////////////////////////////////////////////

CubeSolution::CubeSolution() 
    : iLength(0), iMaxLength(0), iaMoves(NULL), iaTurns(NULL), iInfo(0)
{
}

CubeSolution::CubeSolution(const CubeSolution& sol)
    : iLength(sol.iLength), iMaxLength(0), 
      iaMoves(NULL), iaTurns(NULL), iInfo(sol.iInfo)
{
    if (iLength > 0)
    {
        iMaxLength = ((int) ((iLength-1)/10) + 1) * 10;
        iaMoves = new CubeMove::eMove[iMaxLength];
        iaTurns = new CubeMove::eTurn[iMaxLength];
        for (int i=0; i<iLength; i++)
        {
            iaMoves[i] = sol.iaMoves[i];
            iaTurns[i] = sol.iaTurns[i];
        }
    }
}

CubeSolution::~CubeSolution() 
{
    if (iaMoves != NULL)
        delete [] iaMoves;
    if (iaTurns != NULL)
        delete [] iaTurns;
}

CubeSolution& CubeSolution::operator=(const CubeSolution& sol)
{
    if (this!=&sol)
    {
        if (iaMoves != NULL)
            delete [] iaMoves;
        if (iaTurns != NULL)
            delete [] iaTurns;

        iLength = sol.iLength;
        iMaxLength = ((int) ((iLength-1)/10) + 1) * 10;
        iInfo = sol.iInfo;
    
        iaMoves = NULL;
        iaTurns = NULL;
        if (iLength > 0)
        {
            iaMoves = new CubeMove::eMove[iMaxLength];
            iaTurns = new CubeMove::eTurn[iMaxLength];
            for (int i=0; i<iLength; i++)
            {
                iaMoves[i] = sol.iaMoves[i];
                iaTurns[i] = sol.iaTurns[i];
            }
        }
    }
    return *this;
}

inline void CubeSolution::Extend()
{
    iMaxLength += 10;

    CubeMove::eMove *tmpMoves = new CubeMove::eMove[iMaxLength];     
    CubeMove::eTurn *tmpTurns = new CubeMove::eTurn[iMaxLength];     
    if (iLength > 0)
    {
        memcpy(tmpMoves, iaMoves, iLength * sizeof(int));
        memcpy(tmpTurns, iaTurns, iLength * sizeof(int));
        delete [] iaMoves;
        delete [] iaTurns;
    }
    iaMoves = tmpMoves;
    iaTurns = tmpTurns;
}


////////////////////////////////////////////////////////////////////////////////
// GET/SET STATE PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////
 
void CubeSolution::SetMove(int pos, CubeMove::eMove move, CubeMove::eTurn turn)
     throw (CubeSolutionException)
{
     if (pos < 0 || pos > iLength)
     {
         char *sSol = ToString();
         CubeSolutionException exp(sSol, pos, CubeSolutionException::ERR_CSOL_INVALID_POS);
         delete sSol;
         throw exp;
     }

     if (pos == iMaxLength)
         Extend();

     iaMoves[pos] = move;
     iaTurns[pos] = turn;
     iLength = pos + 1;
}

void CubeSolution::GetMove(int pos, CubeMove::eMove& move, CubeMove::eTurn& turn) const
     throw (CubeSolutionException)
{
     if (pos < 0 || pos >= iLength)
     {
         char *sSol = ToString();
         CubeSolutionException exp(sSol, pos, CubeSolutionException::ERR_CSOL_INVALID_POS);
         delete sSol;
         throw exp;
     }

     move = iaMoves[pos];
     turn = iaTurns[pos];
}
    
void CubeSolution::FromString(const char* sSol) 
     throw (Exception)
{
    // Check format, move names and turns in string
    CheckFormat(sSol);
     
    // delete current solution
    if (iaMoves != NULL)
        delete [] iaMoves;
    if (iaTurns != NULL)
        delete [] iaTurns;
    iaMoves = NULL;
    iaTurns = NULL;
    iInfo = 0;

    // set appropriate lengths
    if (sSol != NULL)
        iLength = (strlen(sSol) + 1) / 3;
    else
        iLength = 0;
    iMaxLength = ((int) ((iLength-1)/10) + 1) * 10;

    if (iLength > 0)
    {
        iaMoves = new CubeMove::eMove[iMaxLength];
        iaTurns = new CubeMove::eTurn[iMaxLength];
        const char DELIMS[] = { DELIM, '\0' };
        char *sol = new char[strlen(sSol)+1];
        strncpy(sol, sSol, strlen(sSol)+1);
        char *ptr = strtok(sol, DELIMS);
        for (int i=0; i<iLength && ptr!=NULL; i++)
        {
            iaMoves[i] = CubeMove::MOVE_NUMBER_FROM_NAME(ptr[0]);
            iaTurns[i] = CubeMove::TURN_NUMBER_FROM_NAME(ptr[1]);
            ptr = strtok(NULL, DELIMS);
        }
        delete[] sol;
    }
    else
        iInfo = INFO_ALREADY_SOLVED;
}

char* CubeSolution::ToString() const
{
    if (iLength == 0)
    {
        char *sSol = new char[1];
        sSol[0] = '\0';
        return sSol;
    }

    char *sSol = new char[3*iLength];

    // Fill moves
    for (int i=0; i<iLength; i++) 
    {
        sSol[3*i] = CubeMove::MOVE_NAMES[iaMoves[i]];
        sSol[3*i + 1] = CubeMove::TURN_NAMES[iaTurns[i]];
        sSol[3*i + 2] = DELIM;
    }
    sSol[3*iLength - 1] = '\0';
    return sSol;
}


////////////////////////////////////////////////////////////////////////////////
// PUBLIC METHOD 
////////////////////////////////////////////////////////////////////////////////

void CubeSolution::SetInfo(const eInfo info, bool on)
{
    iInfo = (info & ~(1 << info)) | (( on ? 1 : 0 ) << info);
}

bool CubeSolution::GetInfo(const eInfo info) const
{
    return (iInfo >> info) & 1 == 1;
}


////////////////////////////////////////////////////////////////////////////////
// PUBLIC METHOD 
////////////////////////////////////////////////////////////////////////////////

bool CubeSolution::IsAllowed()
{
    // forbid moving of the same face twice in a row
    if (iLength > 1
     && iaMoves[iLength-2] == iaMoves[iLength-1])
        return false;
        
    // forbid moving of opposite face after move of B,L albo D. 
    // (BF, LR, DU is the same as FB,RL,UD)
    if (iLength > 1
     && iaMoves[iLength-2] > 3 
     && iaMoves[iLength-2] == CubeMove::OPPOSITE_FACE_MOVE(iaMoves[iLength-1]))
        return false;
        
    // forbid third move of opposite faces
    // (UDU is the same as DU2 and U2D)
    if (iLength > 2 
     && iaMoves[iLength-3] == iaMoves[iLength-1] 
     && iaMoves[iLength-2] == CubeMove::OPPOSITE_FACE_MOVE(iaMoves[iLength-1]))
        return false;
    
    return true;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE CHECK
////////////////////////////////////////////////////////////////////////////////

void CubeSolution::CheckFormat(const char* sSol)
     throw (CubeSolutionException)
{
    int i;
     
    // Check if sSol exists
    if (sSol==NULL)
        return;
    
    // Check format, move names and turns of the string
    int len = strlen(sSol);
    for (i=0; i<len; i++)
    {
        // Check if move name is valid
        if (i%3 == 0)
        {
            if (strchr(CubeMove::MOVE_NAMES, sSol[i]) == NULL)
                throw CubeSolutionException(sSol, i, 
                    CubeSolutionException::ERR_CSOL_PARSE_MOVEINVALID);
        }
        // Check if move turn is valid
        else if (i%3 == 1)
        {
            if (strchr(CubeMove::TURN_NAMES, sSol[i]) == NULL)
                throw CubeSolutionException(sSol, i, 
                    CubeSolutionException::ERR_CSOL_PARSE_TURNINVALID);
        }
        // Check if format is valid
        else
        {
            if (sSol[i] != ',')
                throw CubeSolutionException(sSol, i, 
                    CubeSolutionException::ERR_CSOL_PARSE_FORMATINVALID);
        }
    }

    // check if proper length
    if (len%3 != 2)
        throw CubeSolutionException(sSol, i, 
            CubeSolutionException::ERR_CSOL_PARSE_FORMATINVALID);
}
